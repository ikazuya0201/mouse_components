use core::marker::PhantomData;

use nb::block;
use uom::si::{
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularVelocity, Frequency, Length, Time,
        Velocity,
    },
    ratio::ratio,
};

use crate::robot::StateEstimator;
use crate::tracker::State;
use crate::traits::Math;
use crate::wall_detector::CorrectInfo;

pub trait IMU {
    type Error;

    fn get_translational_acceleration(&mut self) -> nb::Result<Acceleration, Self::Error>;
    fn get_angular_velocity(&mut self) -> nb::Result<AngularVelocity, Self::Error>;
}

pub trait Encoder {
    type Error;

    fn get_relative_distance(&mut self) -> nb::Result<Length, Self::Error>;
}

pub struct Estimator<LE, RE, I, M> {
    period: Time,
    alpha: f32,
    trans_velocity: Velocity,
    angular_velocity: AngularVelocity,
    bias: AngularVelocity,
    wheel_interval: Option<Length>,
    left_encoder: LE,
    right_encoder: RE,
    imu: I,
    initial_state: State,
    state: State,
    weight: f32,
    _phantom: PhantomData<fn() -> M>,
}

impl<LE, RE, I, M> core::fmt::Debug for Estimator<LE, RE, I, M> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(
            f,
            "Estimator {{ state:{:?}, trans_velocity:{:?}, angular_velocity:{:?}, bias:{:?} }}",
            self.state, self.trans_velocity, self.angular_velocity, self.bias,
        )
    }
}

impl<LE, RE, I, M> Estimator<LE, RE, I, M> {
    pub fn init(&mut self) {
        self.state = self.initial_state.clone();
        self.trans_velocity = Default::default();
        self.angular_velocity = Default::default();
        self.bias = Default::default();
    }
}

impl<LE, RE, I, M> StateEstimator<CorrectInfo> for Estimator<LE, RE, I, M>
where
    LE: Encoder,
    RE: Encoder,
    I: IMU,
    <LE as Encoder>::Error: core::fmt::Debug,
    <RE as Encoder>::Error: core::fmt::Debug,
    <I as IMU>::Error: core::fmt::Debug,
    M: Math,
{
    type State = State;

    fn state(&self) -> &State {
        &self.state
    }

    fn estimate(&mut self) {
        //velocity estimation
        let left_distance = block!(self.left_encoder.get_relative_distance()).unwrap();
        let right_distance = block!(self.right_encoder.get_relative_distance()).unwrap();

        let average_left_velocity = left_distance / self.period;
        let average_right_velocity = right_distance / self.period;

        let average_trans_velocity = (average_left_velocity + average_right_velocity) / 2.0;

        let trans_acceleration = block!(self.imu.get_translational_acceleration()).unwrap();
        let imu_angular_velocity = block!(self.imu.get_angular_velocity()).unwrap();

        let angular_velocity = if let Some(wheel_interval) = self.wheel_interval {
            let average_angular_velocity = AngularVelocity::from(
                (average_right_velocity - average_left_velocity) / wheel_interval,
            );

            self.bias = self.alpha * self.bias
                + (1.0 - self.alpha) * (imu_angular_velocity - average_angular_velocity);

            imu_angular_velocity - self.bias
        } else {
            imu_angular_velocity
        };

        //complementary filter
        let trans_velocity = self.alpha * (self.trans_velocity + trans_acceleration * self.period)
            + (1.0 - self.alpha) * average_trans_velocity;
        //------

        //pose estimation
        let trans_distance = trans_velocity * self.period;
        let angle = Angle::from(angular_velocity * self.period);
        let middle_theta = self.state.theta.x + angle / 2.0;
        let (sin_mth, cos_mth) = M::sincos(middle_theta);
        self.state.x.x += trans_distance * cos_mth;
        self.state.y.x += trans_distance * sin_mth;

        self.state.theta.x += angle;
        //------

        let (sin_th, cos_th) = M::sincos(self.state.theta.x);
        self.state.x.v = trans_velocity * cos_th;
        self.state.y.v = trans_velocity * sin_th;
        self.state.x.a = trans_acceleration * cos_th;
        self.state.y.a = trans_acceleration * sin_th;

        self.state.theta.v = angular_velocity;
        self.state.theta.a =
            AngularAcceleration::from((angular_velocity - self.angular_velocity) / self.period);

        self.trans_velocity = trans_velocity;
        self.angular_velocity = angular_velocity;
    }

    fn correct_state<Infos: IntoIterator<Item = CorrectInfo>>(&mut self, infos: Infos) {
        let mut sum_x = Length::default();
        let mut sum_y = Length::default();
        for info in infos {
            let (sin, cos) = M::sincos(info.obstacle.source.theta);
            sum_x += info.diff_from_expected * cos;
            sum_y += info.diff_from_expected * sin;
        }
        self.state.x.x -= self.weight * sum_x;
        self.state.y.x -= self.weight * sum_y;
    }
}

pub struct EstimatorBuilder<LeftEncoder, RightEncoder, Imu> {
    left_encoder: Option<LeftEncoder>,
    right_encoder: Option<RightEncoder>,
    imu: Option<Imu>,
    period: Option<Time>,
    cut_off_frequency: Option<Frequency>,
    initial_state: Option<State>,
    wheel_interval: Option<Length>,
    correction_weight: Option<f32>,
}

impl<LeftEncoder: Encoder, RightEncoder, Imu> EstimatorBuilder<LeftEncoder, RightEncoder, Imu> {
    pub fn left_encoder(mut self, left_encoder: LeftEncoder) -> Self {
        self.left_encoder = Some(left_encoder);
        self
    }
}

impl<LeftEncoder, RightEncoder: Encoder, Imu> EstimatorBuilder<LeftEncoder, RightEncoder, Imu> {
    pub fn right_encoder(mut self, right_encoder: RightEncoder) -> Self {
        self.right_encoder = Some(right_encoder);
        self
    }
}

impl<LeftEncoder, RightEncoder, Imu: IMU> EstimatorBuilder<LeftEncoder, RightEncoder, Imu> {
    pub fn imu(mut self, imu: Imu) -> Self {
        self.imu = Some(imu);
        self
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
            wheel_interval: None,
            correction_weight: None,
        }
    }

    pub fn period(mut self, period: Time) -> Self {
        self.period = Some(period);
        self
    }

    pub fn cut_off_frequency(mut self, cut_off_frequency: Frequency) -> Self {
        self.cut_off_frequency = Some(cut_off_frequency);
        self
    }

    pub fn initial_state(mut self, initial_state: State) -> Self {
        self.initial_state = Some(initial_state);
        self
    }

    pub fn wheel_interval(mut self, wheel_interval: Length) -> Self {
        self.wheel_interval = Some(wheel_interval);
        self
    }

    pub fn correction_weight(mut self, weight: f32) -> Self {
        self.correction_weight = Some(weight);
        self
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BuildError {
    field_name: &'static str,
}

fn ok_or<T>(value: Option<T>, field_name: &'static str) -> Result<T, BuildError> {
    value.ok_or(BuildError { field_name })
}

impl<LeftEncoder: Encoder, RightEncoder: Encoder, Imu: IMU>
    EstimatorBuilder<LeftEncoder, RightEncoder, Imu>
{
    pub fn build<M: Math>(
        self,
    ) -> Result<Estimator<LeftEncoder, RightEncoder, Imu, M>, BuildError> {
        let period = ok_or(self.period, "period")?;
        let cut_off_frequency = ok_or(self.cut_off_frequency, "cut_off_frequency")?;
        let alpha =
            1.0 / (2.0 * core::f32::consts::PI * (period * cut_off_frequency).get::<ratio>() + 1.0);

        let initial_state = self.initial_state.unwrap_or(Default::default());
        Ok(Estimator {
            period,
            alpha,
            trans_velocity: Default::default(),
            angular_velocity: Default::default(),
            left_encoder: ok_or(self.left_encoder, "left_encoder")?,
            right_encoder: ok_or(self.right_encoder, "right_encoder")?,
            imu: ok_or(self.imu, "imu")?,
            bias: Default::default(),
            wheel_interval: self.wheel_interval,
            initial_state: initial_state.clone(),
            state: initial_state,
            weight: self.correction_weight.unwrap_or(0.0),
            _phantom: PhantomData,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        prelude::*,
        trajectory_generators::{
            slalom_parameters_map, SearchTrajectoryGenerator, SearchTrajectoryGeneratorBuilder,
        },
        types::data::{AngleState, LengthState, Pose, SearchKind, Target},
        utils::math::MathFake,
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

    const EPSILON: f32 = 1e-2;

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
        fn step(&self) -> Result<State, FinishError> {
            self.inner.borrow_mut().step()
        }
    }

    struct AgentSimulatorInner<T> {
        trajectory: T,
        current: State,
        prev: State,
    }

    #[derive(Clone, Copy, Debug)]
    struct FinishError;

    impl<T> AgentSimulatorInner<T>
    where
        T: Iterator<Item = Target>,
    {
        fn step(&mut self) -> Result<State, FinishError> {
            if let Some(target) = self.trajectory.next() {
                let state = match target {
                    Target::Moving(target) => State {
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
                    },
                    Target::Spin(target) => State {
                        x: Default::default(),
                        y: Default::default(),
                        theta: AngleState {
                            x: target.x,
                            v: target.v,
                            a: target.a,
                        },
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
            let d = xd * MathFake::cos(middle_angle) + yd * MathFake::sin(middle_angle);
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

    fn build_generator() -> SearchTrajectoryGenerator<MathFake> {
        SearchTrajectoryGeneratorBuilder::new()
            .max_velocity(Velocity::new::<meter_per_second>(1.0))
            .max_acceleration(Acceleration::new::<meter_per_second_squared>(10.0))
            .max_jerk(Jerk::new::<meter_per_second_cubed>(100.0))
            .angular_velocity_ref(AngularVelocity::new::<degree_per_second>(540.0))
            .angular_acceleration_ref(AngularAcceleration::new::<degree_per_second_squared>(
                6480.0,
            ))
            .angular_jerk_ref(AngularJerk::new::<degree_per_second_cubed>(216000.0))
            .slalom_parameters_map(slalom_parameters_map)
            .period(PERIOD)
            .search_velocity(Velocity::new::<meter_per_second>(0.6))
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
                let mut estimator = EstimatorBuilder::new()
                    .left_encoder(left_encoder)
                    .right_encoder(right_encoder)
                    .imu(imu)
                    .period(PERIOD)
                    .cut_off_frequency(Frequency::new::<hertz>(50.0))
                    .wheel_interval(wheel_interval)
                    .build::<MathFake>()
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
