//! An implementation of [Estimator](crate::robot::Estimator).

use core::marker::PhantomData;

#[allow(unused_imports)]
use micromath::F32Ext;
use nb::block;
use serde::{Deserialize, Serialize};
use uom::si::{
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularVelocity, Frequency, Length, Time,
        Velocity,
    },
    ratio::ratio,
};

use crate::robot::Estimator as IEstimator;
use crate::tracker::RobotState;
use crate::utils::builder::{ok_or, BuilderResult};
use crate::{get_or_err, Construct, Deconstruct};

pub trait IMU {
    type Error;

    fn get_translational_acceleration(&mut self) -> nb::Result<Acceleration, Self::Error>;
    fn get_angular_velocity(&mut self) -> nb::Result<AngularVelocity, Self::Error>;
}

pub trait Encoder {
    type Error;

    fn get_relative_distance(&mut self) -> nb::Result<Length, Self::Error>;
}

pub struct EstimatorInner {
    period: Time,
    alpha: f32,
    trans_velocity: Velocity,
    state: RobotState,
    slip_angle_const: Acceleration,
    slip_angle: Angle,
    approximation_threshold: AngularVelocity,
}

impl EstimatorInner {
    pub const DEFAULT_APPROXIMATION_THRESHOLD: AngularVelocity = AngularVelocity {
        value: 0.1,
        dimension: PhantomData,
        units: PhantomData,
    };

    pub fn new(
        period: Time,
        alpha: f32,
        initial_state: RobotState,
        slip_angle_const: Acceleration,
        approximation_threshold: AngularVelocity,
    ) -> Self {
        Self {
            period,
            alpha,
            trans_velocity: Default::default(),
            state: initial_state,
            slip_angle_const,
            slip_angle: Default::default(),
            approximation_threshold,
        }
    }

    pub fn estimate(
        &mut self,
        left_distance: Length,
        right_distance: Length,
        translational_acceleration: Acceleration,
        angular_velocity: AngularVelocity,
    ) {
        //velocity estimation
        let average_left_velocity = left_distance / self.period;
        let average_right_velocity = right_distance / self.period;

        let average_trans_velocity = (average_left_velocity + average_right_velocity) / 2.0;

        //complementary filter
        self.trans_velocity = self.alpha
            * (self.trans_velocity + translational_acceleration * self.period)
            + (1.0 - self.alpha) * average_trans_velocity;
        //------

        //pose estimation
        self.slip_angle = {
            let rev_period = 1.0 / self.period;
            Angle::from(
                (AngularVelocity::from(self.slip_angle * rev_period) + angular_velocity)
                    / (rev_period + self.slip_angle_const / self.trans_velocity),
            )
        };

        let dtheta = Angle::from(angular_velocity * self.period);
        let trans_distance = if angular_velocity.abs() < self.approximation_threshold {
            // straight approximation
            self.trans_velocity * self.period
        } else {
            // arc approximation
            2.0 * self.trans_velocity * (dtheta / 2.0).value.sin() / angular_velocity
        };

        let theta_m = self.state.theta.x - self.slip_angle + dtheta / 2.0;
        let sin_th = theta_m.value.sin();
        let cos_th = theta_m.value.cos();
        self.state.x.x += trans_distance * cos_th;
        self.state.y.x += trans_distance * sin_th;
        self.state.theta.x += dtheta;
        //------

        self.state.x.v = self.trans_velocity * cos_th;
        self.state.y.v = self.trans_velocity * sin_th;
        self.state.x.a = translational_acceleration * cos_th;
        self.state.y.a = translational_acceleration * sin_th;

        self.state.theta.a =
            AngularAcceleration::from((angular_velocity - self.state.theta.v) / self.period);
        self.state.theta.v = angular_velocity;
    }
}

/// An implementation of [Estimator](crate::robot::Estimator).
///
/// This should be created by [EstimatorBuilder](EstimatorBuilder).
pub struct Estimator<LE, RE, I> {
    inner: EstimatorInner,
    left_encoder: LE,
    right_encoder: RE,
    imu: I,
}

impl<LE, RE, I> core::fmt::Debug for Estimator<LE, RE, I> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(
            f,
            "Estimator {{ state:{:?}, trans_velocity:{:?}, angular_velocity:{:?} }}",
            self.inner.state, self.inner.trans_velocity, self.inner.state.theta.v,
        )
    }
}

impl<LE, RE, I> Estimator<LE, RE, I> {
    pub fn release(self) -> (LE, RE, I, RobotState) {
        let Self {
            left_encoder,
            right_encoder,
            imu,
            inner,
            ..
        } = self;
        let EstimatorInner { state, .. } = inner;
        (left_encoder, right_encoder, imu, state)
    }
}

/// Config for [Estimator].
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct EstimatorConfig {
    pub period: Time,
    pub cut_off_frequency: Frequency,
    pub slip_angle_const: Acceleration,
    pub approximation_threshold: AngularVelocity,
}

/// State for [Estimator].
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct EstimatorState {
    pub robot_state: RobotState,
}

/// Resource for [Estimator].
#[derive(PartialEq, Eq, Debug)]
pub struct EstimatorResource<LeftEncoder, RightEncoder, Imu> {
    pub left_encoder: LeftEncoder,
    pub right_encoder: RightEncoder,
    pub imu: Imu,
}

impl<LeftEncoder, RightEncoder, Imu, Config, State, Resource> Construct<Config, State, Resource>
    for Estimator<LeftEncoder, RightEncoder, Imu>
where
    LeftEncoder: Encoder,
    RightEncoder: Encoder,
    Imu: IMU,
    Config: AsRef<EstimatorConfig>,
    State: AsRef<EstimatorState>,
    Resource: AsMut<Option<EstimatorResource<LeftEncoder, RightEncoder, Imu>>>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let config = config.as_ref();
        let state = state.as_ref();
        let EstimatorResource {
            left_encoder,
            right_encoder,
            imu,
        } = resource.as_mut().take().unwrap_or_else(|| unimplemented!());
        EstimatorBuilder::new()
            .left_encoder(left_encoder)
            .right_encoder(right_encoder)
            .imu(imu)
            .initial_state(state.robot_state.clone())
            .period(config.period)
            .cut_off_frequency(config.cut_off_frequency)
            .slip_angle_const(config.slip_angle_const)
            .approximation_threshold(config.approximation_threshold)
            .build()
            .expect("Should never panic")
    }
}

impl<LeftEncoder, RightEncoder, Imu, State, Resource> Deconstruct<State, Resource>
    for Estimator<LeftEncoder, RightEncoder, Imu>
where
    State: From<EstimatorState>,
    Resource: From<EstimatorResource<LeftEncoder, RightEncoder, Imu>>,
{
    fn deconstruct(self) -> (State, Resource) {
        let (left_encoder, right_encoder, imu, robot_state) = self.release();
        (
            EstimatorState { robot_state }.into(),
            EstimatorResource {
                left_encoder,
                right_encoder,
                imu,
            }
            .into(),
        )
    }
}

impl<LE, RE, I> IEstimator for Estimator<LE, RE, I>
where
    LE: Encoder,
    RE: Encoder,
    I: IMU,
    <LE as Encoder>::Error: core::fmt::Debug,
    <RE as Encoder>::Error: core::fmt::Debug,
    <I as IMU>::Error: core::fmt::Debug,
{
    type State = RobotState;

    fn state(&self) -> &RobotState {
        &self.inner.state
    }

    fn estimate(&mut self) {
        //velocity estimation
        let left_distance = block!(self.left_encoder.get_relative_distance()).unwrap();
        let right_distance = block!(self.right_encoder.get_relative_distance()).unwrap();

        let trans_acceleration = block!(self.imu.get_translational_acceleration()).unwrap();
        let imu_angular_velocity = block!(self.imu.get_angular_velocity()).unwrap();

        self.inner.estimate(
            left_distance,
            right_distance,
            trans_acceleration,
            imu_angular_velocity,
        )
    }
}

//TODO: Use a testable example.
/// A builder for [Estimator](Estimator).
///
/// # Example
///
/// ```text
/// use components::estimator::EstimatorBuilder;
/// use uom::si::f32::{Length, Frequency, Time};
/// use uom::si::{length::meter, frequency::hertz, time::second};
///
/// let mut estimator = EstimatorBuilder::new()
///     .left_encoder(left_encoder)
///     .right_encoder(right_encoder)
///     .imu(imu)
///     .period(Time::new::<second>(0.001))
///     .cut_off_frequency(Frequency::new::<hertz>(50.0))
///     .correction_weight(0.1)
///     .build::<Math>()
///     .unwrap();
/// ```
pub struct EstimatorBuilder<LeftEncoder, RightEncoder, Imu> {
    left_encoder: Option<LeftEncoder>,
    right_encoder: Option<RightEncoder>,
    imu: Option<Imu>,
    period: Option<Time>,
    cut_off_frequency: Option<Frequency>,
    initial_state: Option<RobotState>,
    slip_angle_const: Option<Acceleration>,
    approximation_threshold: Option<AngularVelocity>,
}

impl<LeftEncoder: Encoder, RightEncoder, Imu> EstimatorBuilder<LeftEncoder, RightEncoder, Imu> {
    pub fn left_encoder(&mut self, left_encoder: LeftEncoder) -> &mut Self {
        self.left_encoder = Some(left_encoder);
        self
    }
}

impl<LeftEncoder, RightEncoder: Encoder, Imu> EstimatorBuilder<LeftEncoder, RightEncoder, Imu> {
    pub fn right_encoder(&mut self, right_encoder: RightEncoder) -> &mut Self {
        self.right_encoder = Some(right_encoder);
        self
    }
}

impl<LeftEncoder, RightEncoder, Imu: IMU> EstimatorBuilder<LeftEncoder, RightEncoder, Imu> {
    pub fn imu(&mut self, imu: Imu) -> &mut Self {
        self.imu = Some(imu);
        self
    }
}

impl<LeftEncoder, RightEncoder, Imu> Default for EstimatorBuilder<LeftEncoder, RightEncoder, Imu> {
    fn default() -> Self {
        Self::new()
    }
}

impl<LeftEncoder, RightEncoder, Imu> EstimatorBuilder<LeftEncoder, RightEncoder, Imu> {
    pub fn new() -> Self {
        Self {
            left_encoder: None,
            right_encoder: None,
            imu: None,
            period: None,
            cut_off_frequency: None,
            initial_state: None,
            slip_angle_const: None,
            approximation_threshold: None,
        }
    }

    pub fn period(&mut self, period: Time) -> &mut Self {
        self.period = Some(period);
        self
    }

    pub fn cut_off_frequency(&mut self, cut_off_frequency: Frequency) -> &mut Self {
        self.cut_off_frequency = Some(cut_off_frequency);
        self
    }

    pub fn initial_state(&mut self, initial_state: RobotState) -> &mut Self {
        self.initial_state = Some(initial_state);
        self
    }

    pub fn slip_angle_const(&mut self, slip_angle_const: Acceleration) -> &mut Self {
        self.slip_angle_const = Some(slip_angle_const);
        self
    }

    pub fn approximation_threshold(
        &mut self,
        approximation_threshold: AngularVelocity,
    ) -> &mut Self {
        self.approximation_threshold = Some(approximation_threshold);
        self
    }
}

impl<LeftEncoder: Encoder, RightEncoder: Encoder, Imu: IMU>
    EstimatorBuilder<LeftEncoder, RightEncoder, Imu>
{
    pub fn build(&mut self) -> BuilderResult<Estimator<LeftEncoder, RightEncoder, Imu>> {
        let period = ok_or(self.period, "period")?;
        let cut_off_frequency = ok_or(self.cut_off_frequency, "cut_off_frequency")?;
        let alpha =
            1.0 / (2.0 * core::f32::consts::PI * (period * cut_off_frequency).get::<ratio>() + 1.0);

        Ok(Estimator {
            inner: EstimatorInner::new(
                period,
                alpha,
                self.initial_state.take().unwrap_or(Default::default()),
                get_or_err!(self.slip_angle_const),
                self.approximation_threshold
                    .unwrap_or(EstimatorInner::DEFAULT_APPROXIMATION_THRESHOLD),
            ),
            left_encoder: ok_or(self.left_encoder.take(), "left_encoder")?,
            right_encoder: ok_or(self.right_encoder.take(), "right_encoder")?,
            imu: ok_or(self.imu.take(), "imu")?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        prelude::*,
        trajectory_generators::{
            DefaultSlalomParametersGenerator, SearchTrajectoryGenerator,
            SearchTrajectoryGeneratorBuilder,
        },
        types::data::{AngleState, LengthState, Pose, SearchKind, Target},
    };
    use approx::assert_abs_diff_eq;
    use core::marker::PhantomData;
    use std::cell::RefCell;
    use std::rc::Rc;
    use uom::si::{
        acceleration::meter_per_second_squared, angle::radian,
        angular_acceleration::degree_per_second_squared, angular_jerk::degree_per_second_cubed,
        angular_velocity::degree_per_second, f32::*, frequency::hertz,
        jerk::meter_per_second_cubed, length::meter, velocity::meter_per_second,
    };

    const EPSILON: f32 = 2e-2;

    struct AgentSimulator<T> {
        inner: Rc<RefCell<AgentSimulatorInner<T>>>,
    }

    impl<T> AgentSimulator<T> {
        fn new(trajectory: T) -> Self {
            Self {
                inner: Rc::new(RefCell::new(AgentSimulatorInner {
                    trajectory,
                    current: Default::default(),
                    prev: Default::default(),
                })),
            }
        }

        fn split(&self, wheel_interval: Length) -> (IEncoder<T>, IEncoder<T>, IIMU<T>) {
            (
                IEncoder {
                    inner: Rc::clone(&self.inner),
                    wheel_interval,
                    direction: Direction::Right,
                },
                IEncoder {
                    inner: Rc::clone(&self.inner),
                    wheel_interval,
                    direction: Direction::Left,
                },
                IIMU {
                    inner: Rc::clone(&self.inner),
                },
            )
        }
    }

    impl<T> AgentSimulator<T>
    where
        T: Iterator<Item = Target>,
    {
        fn step(&self) -> Result<RobotState, FinishError> {
            self.inner.borrow_mut().step()
        }
    }

    struct AgentSimulatorInner<T> {
        trajectory: T,
        current: RobotState,
        prev: RobotState,
    }

    #[derive(Clone, Copy, Debug)]
    struct FinishError;

    impl<T> AgentSimulatorInner<T>
    where
        T: Iterator<Item = Target>,
    {
        fn step(&mut self) -> Result<RobotState, FinishError> {
            if let Some(target) = self.trajectory.next() {
                let state = RobotState {
                    x: LengthState {
                        x: target.x.x,
                        v: target.x.v,
                        a: target.x.a,
                    },
                    y: LengthState {
                        x: target.y.x,
                        v: target.y.v,
                        a: target.y.a,
                    },
                    theta: AngleState {
                        x: target.theta.x,
                        v: target.theta.v,
                        a: target.theta.a,
                    },
                };
                core::mem::swap(&mut self.prev, &mut self.current);
                self.current = state.clone();
                Ok(state)
            } else {
                Err(FinishError)
            }
        }
    }

    #[derive(Clone, Copy, Debug)]
    enum Direction {
        Right,
        Left,
    }

    struct IEncoder<T> {
        inner: Rc<RefCell<AgentSimulatorInner<T>>>,
        wheel_interval: Length,
        direction: Direction,
    }

    impl<T> Encoder for IEncoder<T> {
        type Error = ();

        fn get_relative_distance(&mut self) -> nb::Result<Length, Self::Error> {
            let current = self.inner.borrow().current.clone();
            let prev = self.inner.borrow().prev.clone();
            let angle = current.theta.x - prev.theta.x;
            let xd = current.x.x - prev.x.x;
            let yd = current.y.x - prev.y.x;
            let middle_angle = (current.theta.x + prev.theta.x) / 2.0;
            let d = xd * middle_angle.value.cos() + yd * middle_angle.value.sin();
            match self.direction {
                Direction::Right => Ok(d + self.wheel_interval * angle / 2.0),
                Direction::Left => Ok(d - self.wheel_interval * angle / 2.0),
            }
        }
    }

    struct IIMU<T> {
        inner: Rc<RefCell<AgentSimulatorInner<T>>>,
    }

    impl<T> IMU for IIMU<T> {
        type Error = ();

        fn get_angular_velocity(&mut self) -> nb::Result<AngularVelocity, Self::Error> {
            let cur_v = self.inner.borrow().current.theta.v;
            Ok(cur_v)
        }

        fn get_translational_acceleration(&mut self) -> nb::Result<Acceleration, Self::Error> {
            let target = self.inner.borrow().current.clone();
            let cur_a = target.x.a * target.theta.x.get::<radian>().cos()
                + target.y.a * target.theta.x.get::<radian>().sin();
            Ok(cur_a)
        }
    }

    const PERIOD: Time = Time {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.001,
    };

    fn build_generator() -> SearchTrajectoryGenerator {
        SearchTrajectoryGeneratorBuilder::new()
            .max_acceleration(Acceleration::new::<meter_per_second_squared>(10.0))
            .max_jerk(Jerk::new::<meter_per_second_cubed>(100.0))
            .period(PERIOD)
            .search_velocity(Velocity::new::<meter_per_second>(0.6))
            .parameters_generator(DefaultSlalomParametersGenerator::default())
            .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(90.0))
            .spin_angular_acceleration(AngularAcceleration::new::<degree_per_second_squared>(90.0))
            .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(180.0))
            .build()
            .unwrap()
    }

    macro_rules! impl_estimator_test {
        ($name: ident: $trajectory: expr) => {
            #[test]
            fn $name() {
                let wheel_interval = Length::new::<meter>(0.03);

                let trajectory = $trajectory;

                let simulator = AgentSimulator::new(trajectory);
                let (right_encoder, left_encoder, imu) = simulator.split(wheel_interval);
                let mut estimator = EstimatorBuilder::default()
                    .left_encoder(left_encoder)
                    .right_encoder(right_encoder)
                    .imu(imu)
                    .period(PERIOD)
                    .cut_off_frequency(Frequency::new::<hertz>(50.0))
                    .slip_angle_const(Acceleration::new::<meter_per_second_squared>(100.0))
                    .build()
                    .unwrap();

                while let Ok(expected_state) = simulator.step() {
                    estimator.estimate();
                    let estimated_state = estimator.state();
                    assert_abs_diff_eq!(expected_state, estimated_state, epsilon = EPSILON);
                }
            }
        };
    }

    fn straight_trajectory() -> impl Iterator<Item = Target> {
        let trajectory_generator = build_generator();
        trajectory_generator.generate_search(&(Pose::default(), SearchKind::Init))
    }

    fn straight_and_search_trajectory(kind: SearchKind) -> impl Iterator<Item = Target> {
        let trajectory_generator = build_generator();
        trajectory_generator
            .generate_search(&(Pose::default(), SearchKind::Init))
            .chain(trajectory_generator.generate_search(&(
                Pose {
                    x: Length::new::<meter>(0.045),
                    y: Default::default(),
                    theta: Default::default(),
                },
                kind,
            )))
    }

    impl_estimator_test!(test_estimator_straight1: straight_trajectory());
    impl_estimator_test!(
        test_estimator_straight_right: straight_and_search_trajectory(SearchKind::Right)
    );
    impl_estimator_test!(
        test_estimator_straight_left: straight_and_search_trajectory(SearchKind::Left)
    );
    impl_estimator_test!(
        test_estimator_straight2: straight_and_search_trajectory(SearchKind::Front)
    );
}
