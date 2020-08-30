mod encoder;
mod imu;

use nb::block;
use quantities::{Angle, AngularSpeed, Distance, Frequency, Speed, Time};

use crate::agent::StateEstimator;
use crate::tracker::{State, SubState};
pub use encoder::Encoder;
pub use imu::IMU;

pub struct Estimator<LE, RE, I> {
    initial_x: Distance,
    initial_y: Distance,
    initial_theta: Angle,
    x: Distance,
    y: Distance,
    theta: Angle,
    period: Time,
    alpha: f32,
    trans_speed: Speed,
    angular_speed: AngularSpeed,
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
        self.trans_speed = Default::default();
        self.angular_speed = Default::default();
    }

    fn estimate(&mut self) -> State {
        //velocity estimation
        let left_distance = block!(self.left_encoder.get_relative_distance()).unwrap();
        let right_distance = block!(self.right_encoder.get_relative_distance()).unwrap();

        let average_left_speed = left_distance / self.period;
        let average_right_speed = right_distance / self.period;

        let average_trans_speed = (average_left_speed + average_right_speed) / 2.0;

        let trans_acceleration = block!(self.imu.get_translational_acceleration()).unwrap();
        let angular_speed = block!(self.imu.get_angular_speed()).unwrap();

        //complementary filter
        let trans_speed = self.alpha * (self.trans_speed + trans_acceleration * self.period)
            + (1.0 - self.alpha) * average_trans_speed;
        //------

        //pose estimation
        let trans_distance = trans_speed * self.period;
        let middle_theta =
            self.theta + (self.angular_speed + angular_speed / 2.0) * self.period / 2.0;
        self.x += trans_distance * middle_theta.cos();
        self.y += trans_distance * middle_theta.sin();

        self.theta += (self.angular_speed + angular_speed) * self.period / 2.0;
        //------

        let cos_th = self.theta.cos();
        let sin_th = self.theta.sin();
        let vx = trans_speed * cos_th;
        let vy = trans_speed * sin_th;
        let ax = trans_acceleration * cos_th;
        let ay = trans_acceleration * sin_th;

        let angular_acceleration = (angular_speed - self.angular_speed) / self.period;

        self.trans_speed = trans_speed;
        self.angular_speed = angular_speed;

        State {
            x: SubState {
                x: self.x,
                v: vx,
                a: ax,
            },
            y: SubState {
                x: self.y,
                v: vy,
                a: ay,
            },
            theta: SubState {
                x: self.theta,
                v: angular_speed,
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
        let alpha =
            1.0 / (2.0 * core::f32::consts::PI * self.period * self.cut_off_frequency + 1.0);
        Estimator {
            initial_x: Default::default(),
            initial_y: Default::default(),
            initial_theta: self.initial_posture,
            x: Default::default(),
            y: Default::default(),
            theta: self.initial_posture,
            period: self.period,
            alpha,
            trans_speed: Default::default(),
            angular_speed: Default::default(),
            left_encoder: self.left_encoder,
            right_encoder: self.right_encoder,
            imu: self.imu,
        }
    }
}

impl<LE, RE, I> EstimatorBuilder<LE, RE, I, Time, Frequency, Angle, Distance, Distance>
where
    LE: Encoder,
    RE: Encoder,
    I: IMU,
{
    pub fn build(self) -> Estimator<LE, RE, I> {
        let alpha =
            1.0 / (2.0 * core::f32::consts::PI * self.period * self.cut_off_frequency + 1.0);
        Estimator {
            initial_x: self.initial_x,
            initial_y: self.initial_y,
            initial_theta: self.initial_posture,
            x: self.initial_x,
            y: self.initial_y,
            theta: self.initial_posture,
            period: self.period,
            alpha,
            trans_speed: Default::default(),
            angular_speed: Default::default(),
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
        let alpha =
            1.0 / (2.0 * core::f32::consts::PI * self.period * self.cut_off_frequency + 1.0);
        Estimator {
            initial_x: Default::default(),
            initial_y: Default::default(),
            initial_theta: Default::default(),
            x: Default::default(),
            y: Default::default(),
            theta: Default::default(),
            period: self.period,
            alpha,
            trans_speed: Default::default(),
            angular_speed: Default::default(),
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
        initial_x: Distance,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, Distance, Y> {
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
        initial_y: Distance,
    ) -> EstimatorBuilder<LE, RE, I, P, COF, POS, X, Distance> {
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
