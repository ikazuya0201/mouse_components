mod encoder;
mod imu;

use crate::quantities::dimensionless::scalar;
use crate::quantities::f32::{Angle, AngularVelocity, Frequency, Length, Time, Velocity};
use nb::block;

use crate::agent::StateEstimator;
use crate::tracker::{AngleState, LengthState, State};
pub use encoder::Encoder;
pub use imu::IMU;

pub struct Estimator<LE, RE, I> {
    initial_x: Length,
    initial_y: Length,
    initial_theta: Angle,
    x: Length,
    y: Length,
    theta: Angle,
    period: Time,
    alpha: f32,
    trans_velocity: Velocity,
    angular_velocity: AngularVelocity,
    left_encoder: LE,
    right_encoder: RE,
    imu: I,
}

impl<LE, RE, I> StateEstimator<State> for Estimator<LE, RE, I>
where
    LE: Encoder,
    RE: Encoder,
    I: IMU,
    <LE as Encoder>::Error: core::fmt::Debug,
    <RE as Encoder>::Error: core::fmt::Debug,
    <I as IMU>::Error: core::fmt::Debug,
{
    fn init(&mut self) {
        self.x = self.initial_x;
        self.y = self.initial_y;
        self.theta = self.initial_theta;
        self.trans_velocity = Default::default();
        self.angular_velocity = Default::default();
    }

    fn estimate(&mut self) -> State {
        //velocity estimation
        let left_distance = block!(self.left_encoder.get_relative_distance()).unwrap();
        let right_distance = block!(self.right_encoder.get_relative_distance()).unwrap();

        let average_left_velocity = left_distance / self.period;
        let average_right_velocity = right_distance / self.period;

        let average_trans_velocity = (average_left_velocity + average_right_velocity) / 2.0;

        let trans_acceleration = block!(self.imu.get_translational_acceleration()).unwrap();
        let angular_velocity = block!(self.imu.get_angular_velocity()).unwrap();

        //complementary filter
        let trans_velocity = self.alpha * (self.trans_velocity + trans_acceleration * self.period)
            + (1.0 - self.alpha) * average_trans_velocity;
        //------

        //pose estimation
        let trans_distance = trans_velocity * self.period;
        let middle_theta =
            self.theta + (self.angular_velocity + angular_velocity / 2.0) * self.period / 2.0;
        self.x += trans_distance * middle_theta.cos();
        self.y += trans_distance * middle_theta.sin();

        self.theta += (self.angular_velocity + angular_velocity) * self.period / 2.0;
        //------

        let cos_th = self.theta.cos();
        let sin_th = self.theta.sin();
        let vx = trans_velocity * cos_th;
        let vy = trans_velocity * sin_th;
        let ax = trans_acceleration * cos_th;
        let ay = trans_acceleration * sin_th;

        let angular_acceleration = (angular_velocity - self.angular_velocity) / self.period;

        self.trans_velocity = trans_velocity;
        self.angular_velocity = angular_velocity;

        State {
            x: LengthState {
                x: self.x,
                v: vx,
                a: ax,
            },
            y: LengthState {
                x: self.y,
                v: vy,
                a: ay,
            },
            theta: AngleState {
                x: self.theta,
                v: angular_velocity,
                a: angular_acceleration,
            },
        }
    }
}

pub struct EstimatorBuilder<LE, RE, I, P, COF, POS, X, Y> {
    left_encoder: LE,
    right_encoder: RE,
    imu: I,
    period: P,
    cut_off_frequency: COF,
    initial_posture: POS,
    initial_x: X,
    initial_y: Y,
}

impl EstimatorBuilder<(), (), (), (), (), (), (), ()> {
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
        }
    }
}

impl<LE, RE, I> EstimatorBuilder<LE, RE, I, Time, Frequency, Angle, (), ()>
where
    LE: Encoder,
    RE: Encoder,
    I: IMU,
{
    pub fn build(self) -> Estimator<LE, RE, I> {
        let alpha = 1.0
            / (2.0
                * core::f32::consts::PI
                * (self.period * self.cut_off_frequency).get::<scalar>()
                + 1.0);
        Estimator {
            initial_x: Default::default(),
            initial_y: Default::default(),
            initial_theta: self.initial_posture,
            x: Default::default(),
            y: Default::default(),
            theta: self.initial_posture,
            period: self.period,
            alpha,
            trans_velocity: Default::default(),
            angular_velocity: Default::default(),
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
        }
    }
}

impl<LE, RE, I> EstimatorBuilder<LE, RE, I, Time, Frequency, Angle, Length, Length>
where
    LE: Encoder,
    RE: Encoder,
    I: IMU,
{
    pub fn build(self) -> Estimator<LE, RE, I> {
        let alpha = 1.0
            / (2.0
                * core::f32::consts::PI
                * (self.period * self.cut_off_frequency).get::<scalar>()
                + 1.0);
        Estimator {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            initial_theta: self.initial_posture,
            x: self.initial_x,
            y: self.initial_y,
            theta: self.initial_posture,
            period: self.period,
            alpha,
            trans_velocity: Default::default(),
            angular_velocity: Default::default(),
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
        }
    }
}

impl<LE, RE, I> EstimatorBuilder<LE, RE, I, Time, Frequency, (), (), ()>
where
    LE: Encoder,
    RE: Encoder,
    I: IMU,
{
    pub fn build(self) -> Estimator<LE, RE, I> {
        let alpha = 1.0
            / (2.0
                * core::f32::consts::PI
                * (self.period * self.cut_off_frequency).get::<scalar>()
                + 1.0);
        Estimator {
            initial_x: Default::default(),
            initial_y: Default::default(),
            initial_theta: Default::default(),
            x: Default::default(),
            y: Default::default(),
            theta: Default::default(),
            period: self.period,
            alpha,
            trans_velocity: Default::default(),
            angular_velocity: Default::default(),
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
        }
    }
}

impl<RE, I, P, COF, POS, X, Y> EstimatorBuilder<(), RE, I, P, COF, POS, X, Y> {
    pub fn left_encoder<LE>(
        self,
        left_encoder: LE,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, X, Y>
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
        }
    }
}

impl<LE, I, P, COF, POS, X, Y> EstimatorBuilder<LE, (), I, P, COF, POS, X, Y> {
    pub fn right_encoder<RE>(
        self,
        right_encoder: RE,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, X, Y>
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
        }
    }
}

impl<LE, RE, P, COF, POS, X, Y> EstimatorBuilder<LE, RE, (), P, COF, POS, X, Y> {
    pub fn imu<I>(self, imu: I) -> EstimatorBuilder<LE, RE, I, P, COF, POS, X, Y>
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
        }
    }
}

impl<LE, RE, I, COF, POS, X, Y> EstimatorBuilder<LE, RE, I, (), COF, POS, X, Y> {
    pub fn period(self, period: Time) -> EstimatorBuilder<LE, RE, I, Time, COF, POS, X, Y> {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
        }
    }
}

impl<LE, RE, I, P, POS, X, Y> EstimatorBuilder<LE, RE, I, P, (), POS, X, Y> {
    pub fn cut_off_frequency(
        self,
        cut_off_frequency: Frequency,
    ) -> EstimatorBuilder<LE, RE, I, P, Frequency, POS, X, Y> {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency,
            initial_posture: self.initial_posture,
        }
    }
}

impl<LE, RE, I, P, COF, X, Y> EstimatorBuilder<LE, RE, I, P, COF, (), X, Y> {
    pub fn initial_posture(
        self,
        initial_posture: Angle,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, Angle, X, Y> {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture,
        }
    }
}
impl<LE, RE, I, P, COF, POS, Y> EstimatorBuilder<LE, RE, I, P, COF, POS, (), Y> {
    pub fn initial_x(
        self,
        initial_x: Length,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, Length, Y> {
        EstimatorBuilder {
            initial_x,
            initial_y: self.initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
        }
    }
}
impl<LE, RE, I, P, COF, POS, X> EstimatorBuilder<LE, RE, I, P, COF, POS, X, ()> {
    pub fn initial_y(
        self,
        initial_y: Length,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, X, Length> {
        EstimatorBuilder {
            initial_x: self.initial_x,
            initial_y,
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
            period: self.period,
            cut_off_frequency: self.cut_off_frequency,
            initial_posture: self.initial_posture,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        data_types::{Pose, RelativeDirection, Target},
        impls::{TrajectoryGenerator, TrajectoryGeneratorBuilder},
        prelude::*,
    };
    use approx::assert_abs_diff_eq;
    use quantities::*;
    use std::cell::RefCell;
    use std::rc::Rc;

    const EPSILON: f32 = 1e-3;

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

        fn split(&self, wheel_interval: Distance) -> (IEncoder<T>, IEncoder<T>, IIMU<T>) {
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
                    x: SubState {
                        x: target.x.x,
                        v: target.x.v,
                        a: target.x.a,
                    },
                    y: SubState {
                        x: target.y.x,
                        v: target.y.v,
                        a: target.y.a,
                    },
                    theta: SubState {
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
        wheel_interval: Distance,
        direction: Direction,
    }

    impl<T> Encoder for IEncoder<T> {
        type Error = ();

        fn get_relative_distance(&mut self) -> nb::Result<Distance, Self::Error> {
            let current = self.inner.borrow().current.clone();
            let prev = self.inner.borrow().prev.clone();
            let angle = current.theta.x - prev.theta.x;
            let x = (current.x.x - prev.x.x).as_meters();
            let y = (current.y.x - prev.y.x).as_meters();
            let d = Distance::from_meters((x * x + y * y).sqrt());
            match self.direction {
                Direction::Right => Ok(d + self.wheel_interval * angle),
                Direction::Left => Ok(d - self.wheel_interval * angle),
            }
        }
    }

    struct IIMU<T> {
        inner: Rc<RefCell<AgentSimulatorInner<T>>>,
    }

    impl<T> IMU for IIMU<T> {
        type Error = ();

        fn get_angular_speed(&mut self) -> nb::Result<AngularSpeed, Self::Error> {
            let cur_v = self.inner.borrow().current.theta.v;
            Ok(cur_v)
        }

        fn get_translational_acceleration(&mut self) -> nb::Result<Acceleration, Self::Error> {
            let f = |target: Target| {
                target.x.a * target.theta.x.cos() + target.y.a * target.theta.x.sin()
            };
            let cur_a = f(self.inner.borrow().current);
            Ok(cur_a)
        }
    }

    const PERIOD: Time = Time::from_seconds(0.001);

    fn build_generator() -> TrajectoryGenerator {
        TrajectoryGeneratorBuilder::new()
            .max_speed(Speed::from_meter_per_second(1.0))
            .max_acceleration(Acceleration::from_meter_per_second_squared(10.0))
            .max_jerk(Jerk::from_meter_per_second_cubed(100.0))
            .angular_speed_ref(AngularSpeed::from_degree_per_second(180.0))
            .angular_acceleration_ref(AngularAcceleration::from_degree_per_second_squared(1800.0))
            .angular_jerk_ref(AngularJerk::from_degree_per_second_cubed(18000.0))
            .slalom_speed_ref(Speed::from_meter_per_second(0.6))
            .period(PERIOD)
            .search_speed(Speed::from_meter_per_second(0.6))
            .build()
    }

    macro_rules! estimator_tests {
        ($($name: ident: $trajectory: expr,)*) => {
            $(
                #[test]
                fn $name() {
                    let wheel_interval = Distance::from_meters(0.03);

                    let trajectory = $trajectory;

                    let simulator = AgentSimulator::new(trajectory);
                    let (right_encoder, left_encoder, imu) = simulator.split(wheel_interval);
                    let mut estimator = EstimatorBuilder::new()
                        .left_encoder(left_encoder)
                        .right_encoder(right_encoder)
                        .imu(imu)
                        .period(PERIOD)
                        .cut_off_frequency(Frequency::from_hertz(50.0))
                        .initial_posture(Default::default())
                        .initial_x(Default::default())
                        .initial_y(Default::default())
                        .build();

                    while let Ok(expected_state) = simulator.step() {
                        let estimated_state = estimator.estimate();
                        assert_abs_diff_eq!(expected_state, estimated_state, epsilon = EPSILON);
                    }
                }
            )*
        };
    }

    fn straight_trajectory() -> impl Iterator<Item = Target> {
        let trajectory_generator = build_generator();
        trajectory_generator.generate_straight(
            Default::default(),
            Default::default(),
            Distance::from_meters(3.0),
            Default::default(),
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
                Default::default(),
                Default::default(),
                Distance::from_meters(3.0),
                Default::default(),
                Default::default(),
                Speed::from_meter_per_second(0.6),
            )
            .chain(trajectory_generator.generate_search(
                Pose {
                    x: Distance::from_meters(3.0),
                    y: Default::default(),
                    theta: Default::default(),
                },
                direction,
            ))
    }

    estimator_tests! {
        test_estimator1: straight_trajectory(),
        test_estimator2: straight_and_search_trajectory(RelativeDirection::Right),
        test_estimator3: straight_and_search_trajectory(RelativeDirection::Left),
        test_estimator4: straight_and_search_trajectory(RelativeDirection::Front),
    }
}
