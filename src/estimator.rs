use core::marker::PhantomData;

use nb::block;
use uom::si::{
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularVelocity, Frequency, Length, Time,
        Velocity,
    },
    ratio::ratio,
};

use crate::agent::StateEstimator;
use crate::tracker::{AngleState, LengthState, State};
use crate::traits::Math;

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
    wheel_interval: Length,
    left_encoder: LE,
    right_encoder: RE,
    imu: I,
    initial_state: State,
    state: State,
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

impl<LE, RE, I, M> StateEstimator for Estimator<LE, RE, I, M>
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

    fn init(&mut self) {
        self.state = self.initial_state.clone();
        self.trans_velocity = Default::default();
        self.angular_velocity = Default::default();
        self.bias = Default::default();
    }

    fn state(&self) -> State {
        self.state.clone()
    }

    fn estimate(&mut self) {
        //velocity estimation
        let left_distance = block!(self.left_encoder.get_relative_distance()).unwrap();
        let right_distance = block!(self.right_encoder.get_relative_distance()).unwrap();

        let average_left_velocity = left_distance / self.period;
        let average_right_velocity = right_distance / self.period;

        let average_trans_velocity = (average_left_velocity + average_right_velocity) / 2.0;

        let average_angular_velocity = AngularVelocity::from(
            (average_right_velocity - average_left_velocity) / self.wheel_interval,
        );

        let trans_acceleration = block!(self.imu.get_translational_acceleration()).unwrap();
        let imu_angular_velocity = block!(self.imu.get_angular_velocity()).unwrap();

        self.bias = self.alpha * self.bias
            + (1.0 - self.alpha) * (imu_angular_velocity - average_angular_velocity);

        let angular_velocity = imu_angular_velocity - self.bias;

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
}

pub struct EstimatorBuilder<LE, RE, I, P, COF, POS, X, Y, WI> {
    left_encoder: LE,
    right_encoder: RE,
    imu: I,
    period: P,
    cut_off_frequency: COF,
    initial_posture: POS,
    initial_x: X,
    initial_y: Y,
    wheel_interval: WI,
}

impl<LE, RE, I, P, COF, POS, X, Y, WI> EstimatorBuilder<LE, RE, I, P, COF, POS, X, Y, WI> {
    fn initial_state(x: Length, y: Length, theta: Angle) -> State {
        State {
            x: LengthState {
                x,
                ..Default::default()
            },
            y: LengthState {
                x: y,
                ..Default::default()
            },
            theta: AngleState {
                x: theta,
                ..Default::default()
            },
        }
    }
}

impl EstimatorBuilder<(), (), (), (), (), (), (), (), ()> {
    pub fn new() -> Self {
        Self {
            left_encoder: (),
            right_encoder: (),
            imu: (),
            period: (),
            cut_off_frequency: (),
            initial_posture: (),
            initial_x: (),
            initial_y: (),
            wheel_interval: (),
        }
    }
}

impl<LE, RE, I> EstimatorBuilder<LE, RE, I, Time, Frequency, Angle, Length, Length, Length>
where
    LE: Encoder,
    RE: Encoder,
    I: IMU,
{
    pub fn build<M>(self) -> Estimator<LE, RE, I, M>
    where
        M: Math,
    {
        let alpha = 1.0
            / (2.0 * core::f32::consts::PI * (self.period * self.cut_off_frequency).get::<ratio>()
                + 1.0);

        let state = Self::initial_state(self.initial_x, self.initial_y, self.initial_posture);

        Estimator {
            period: self.period,
            alpha,
            trans_velocity: Default::default(),
            angular_velocity: Default::default(),
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            bias: Default::default(),
            wheel_interval: self.wheel_interval,
            initial_state: state.clone(),
            state,
            _phantom: PhantomData,
        }
    }
}

impl<LE, RE, I> EstimatorBuilder<LE, RE, I, Time, Frequency, (), (), (), Length>
where
    LE: Encoder,
    RE: Encoder,
    I: IMU,
{
    pub fn build<M>(self) -> Estimator<LE, RE, I, M>
    where
        M: Math,
    {
        let alpha = 1.0
            / (2.0 * core::f32::consts::PI * (self.period * self.cut_off_frequency).get::<ratio>()
                + 1.0);

        let state = Self::initial_state(Default::default(), Default::default(), Default::default());

        Estimator {
            period: self.period,
            alpha,
            trans_velocity: Default::default(),
            angular_velocity: Default::default(),
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            bias: Default::default(),
            wheel_interval: self.wheel_interval,
            initial_state: state.clone(),
            state,
            _phantom: PhantomData,
        }
    }
}

impl<RE, I, P, COF, POS, X, Y, WI> EstimatorBuilder<(), RE, I, P, COF, POS, X, Y, WI> {
    pub fn left_encoder<LE>(
        self,
        left_encoder: LE,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, X, Y, WI>
    where
        LE: Encoder,
    {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}

impl<LE, I, P, COF, POS, X, Y, WI> EstimatorBuilder<LE, (), I, P, COF, POS, X, Y, WI> {
    pub fn right_encoder<RE>(
        self,
        right_encoder: RE,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, X, Y, WI>
    where
        RE: Encoder,
    {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}

impl<LE, RE, P, COF, POS, X, Y, WI> EstimatorBuilder<LE, RE, (), P, COF, POS, X, Y, WI> {
    pub fn imu<I>(self, imu: I) -> EstimatorBuilder<LE, RE, I, P, COF, POS, X, Y, WI>
    where
        I: IMU,
    {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}

impl<LE, RE, I, COF, POS, X, Y, WI> EstimatorBuilder<LE, RE, I, (), COF, POS, X, Y, WI> {
    pub fn period(self, period: Time) -> EstimatorBuilder<LE, RE, I, Time, COF, POS, X, Y, WI> {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}

impl<LE, RE, I, P, POS, X, Y, WI> EstimatorBuilder<LE, RE, I, P, (), POS, X, Y, WI> {
    pub fn cut_off_frequency(
        self,
        cut_off_frequency: Frequency,
    ) -> EstimatorBuilder<LE, RE, I, P, Frequency, POS, X, Y, WI> {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}

impl<LE, RE, I, P, COF, X, Y, WI> EstimatorBuilder<LE, RE, I, P, COF, (), X, Y, WI> {
    pub fn initial_posture(
        self,
        initial_posture: Angle,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, Angle, X, Y, WI> {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}
impl<LE, RE, I, P, COF, POS, Y, WI> EstimatorBuilder<LE, RE, I, P, COF, POS, (), Y, WI> {
    pub fn initial_x(
        self,
        initial_x: Length,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, Length, Y, WI> {
        EstimatorBuilder {
            initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}

impl<LE, RE, I, P, COF, POS, X, WI> EstimatorBuilder<LE, RE, I, P, COF, POS, X, (), WI> {
    pub fn initial_y(
        self,
        initial_y: Length,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, X, Length, WI> {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval: self.wheel_interval,
        }
    }
}

impl<LE, RE, I, P, COF, POS, X, Y> EstimatorBuilder<LE, RE, I, P, COF, POS, X, Y, ()> {
    pub fn wheel_interval(
        self,
        wheel_interval: Length,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, X, Y, Length> {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
            wheel_interval,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        data_types::{Pose, RelativeDirection, SearchKind, Target},
        impls::{TrajectoryGenerator, TrajectoryGeneratorBuilder},
        prelude::*,
        trajectory_generator::slalom_parameters_map,
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
        current: Target,
        prev: Target,
    }

    #[derive(Clone, Copy, Debug)]
    struct FinishError;

    impl<T> AgentSimulatorInner<T>
    where
        T: Iterator<Item = Target>,
    {
        fn step(&mut self) -> Result<State, FinishError> {
            if let Some(target) = self.trajectory.next() {
                self.prev = self.current;
                self.current = target;
                Ok(State {
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
                })
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
            let f = |target: Target| {
                target.x.a * target.theta.x.get::<radian>().cos()
                    + target.y.a * target.theta.x.get::<radian>().sin()
            };
            let cur_a = f(self.inner.borrow().current);
            Ok(cur_a)
        }
    }

    const PERIOD: Time = Time {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.001,
    };

    fn build_generator() -> TrajectoryGenerator<MathFake, ()> {
        TrajectoryGeneratorBuilder::new()
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
            .run_slalom_velocity(Velocity::new::<meter_per_second>(1.0))
            .build()
    }

    macro_rules! estimator_tests {
        ($($name: ident: $trajectory: expr,)*) => {
            $(
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
                        .initial_posture(Default::default())
                        .initial_x(Default::default())
                        .initial_y(Default::default())
                        .wheel_interval(wheel_interval)
                        .build::<MathFake>();

                    while let Ok(expected_state) = simulator.step() {
                        estimator.estimate();
                        let estimated_state = estimator.state();
                        assert_abs_diff_eq!(expected_state, estimated_state, epsilon = EPSILON);
                    }
                }
            )*
        };
    }

    fn straight_trajectory() -> impl Iterator<Item = Target> {
        let trajectory_generator = build_generator();
        trajectory_generator.generate_straight(
            Length::new::<meter>(3.0),
            Default::default(),
            Default::default(),
        )
    }

    fn straight_and_search_trajectory(
        direction: RelativeDirection,
    ) -> impl Iterator<Item = Target> {
        let trajectory_generator = build_generator();
        trajectory_generator
            .generate_straight(
                Length::new::<meter>(0.09),
                Default::default(),
                Velocity::new::<meter_per_second>(0.6),
            )
            .chain(trajectory_generator.generate_search(
                &Pose {
                    x: Length::new::<meter>(0.09),
                    y: Default::default(),
                    theta: Default::default(),
                },
                &SearchKind::Search(direction),
            ))
    }

    estimator_tests! {
        test_estimator_straight1: straight_trajectory(),
        test_estimator_straight_right: straight_and_search_trajectory(RelativeDirection::Right),
        test_estimator_straight_left: straight_and_search_trajectory(RelativeDirection::Left),
        test_estimator_straight2: straight_and_search_trajectory(RelativeDirection::Front),
    }
}
