#[macro_use]
extern crate typenum;

mod sensors {
    use components::{
        data_types::Pose,
        sensors::{DistanceSensor, Encoder, Motor, IMU},
        utils::sample::Sample,
    };
    use quantities::{Acceleration, AngularSpeed, Distance, Voltage};

    pub struct IDistanceSensor {
        distance: Distance,
        pose: Pose,
    }

    impl IDistanceSensor {
        const SIGMA: Distance = Distance::from_meters(0.001);

        pub fn new(distance: Distance, pose: Pose) -> Self {
            Self { distance, pose }
        }
    }

    impl DistanceSensor for IDistanceSensor {
        type Error = ();

        fn pose(&self) -> Pose {
            self.pose
        }

        fn get_distance(&mut self) -> nb::Result<Sample<Distance>, Self::Error> {
            Ok(Sample {
                mean: self.distance,
                standard_deviation: Self::SIGMA,
            })
        }
    }

    pub struct IEncoder {
        distance: Distance,
    }

    impl IEncoder {
        pub fn new(distance: Distance) -> Self {
            Self { distance }
        }
    }

    impl Encoder for IEncoder {
        type Error = ();

        fn get_relative_distance(&mut self) -> nb::Result<Distance, Self::Error> {
            Ok(self.distance)
        }
    }

    pub struct IMotor;

    impl Motor for IMotor {
        fn apply(&mut self, _voltage: Voltage) {}
    }

    pub struct Imu {
        accelerations: [Acceleration; 3],
        angular_speeds: [AngularSpeed; 3],
    }

    impl Imu {
        pub fn new(accelerations: [Acceleration; 3], angular_speeds: [AngularSpeed; 3]) -> Self {
            Self {
                accelerations,
                angular_speeds,
            }
        }
    }

    impl IMU for Imu {
        type Error = ();

        fn get_acceleration_x(&mut self) -> nb::Result<Acceleration, Self::Error> {
            Ok(self.accelerations[0])
        }
        fn get_acceleration_y(&mut self) -> nb::Result<Acceleration, Self::Error> {
            Ok(self.accelerations[1])
        }
        fn get_acceleration_z(&mut self) -> nb::Result<Acceleration, Self::Error> {
            Ok(self.accelerations[2])
        }

        fn get_angular_speed_x(&mut self) -> nb::Result<AngularSpeed, Self::Error> {
            Ok(self.angular_speeds[0])
        }
        fn get_angular_speed_y(&mut self) -> nb::Result<AngularSpeed, Self::Error> {
            Ok(self.angular_speeds[1])
        }
        fn get_angular_speed_z(&mut self) -> nb::Result<AngularSpeed, Self::Error> {
            Ok(self.angular_speeds[2])
        }
    }
}

use std::f32::consts::PI;
use std::rc::Rc;

use components::{
    data_types::{AbsoluteDirection, NodeId, Pattern, Pose, SearchNodeId},
    defaults::{DefaultSearchOperator, DefaultSolver},
    impls::{
        Agent, ControllerBuilder, EstimatorBuilder, MazeBuilder, NullLogger, ObstacleDetector,
        TrackerBuilder, TrajectoryGeneratorBuilder,
    },
    prelude::*,
};
use criterion::*;
use generic_array::arr;
use quantities::{
    Acceleration, Angle, AngularAcceleration, AngularJerk, AngularSpeed, Distance, Frequency, Jerk,
    Speed, Time,
};
use typenum::consts::*;

use sensors::{IDistanceSensor, IEncoder, IMotor, Imu};

type MazeWidth = U4;
type MaxSize = op!(U16 * MazeWidth * MazeWidth);
type GoalSize = U2;
type DistanceSensorNum = U3;

fn costs(_pattern: Pattern) -> u16 {
    0
}

fn bench_tick(c: &mut Criterion) {
    c.bench_function("bench_tick", |b| {
        b.iter_batched(
            || {
                let operator = create_search_operator();
                operator.run().ok();
                operator
            },
            |operator| operator.tick(),
            BatchSize::SmallInput,
        );
    });
}

fn bench_run(c: &mut Criterion) {
    c.bench_function("bench_run", |b| {
        b.iter_batched(
            || {
                let operator = create_search_operator();
                operator
            },
            |operator| operator.run().ok(),
            BatchSize::SmallInput,
        );
    });
}

criterion_group!(benches, bench_tick, bench_run);
criterion_main!(benches);

fn create_search_operator() -> DefaultSearchOperator<
    IMotor,
    IMotor,
    IEncoder,
    IEncoder,
    Imu,
    IDistanceSensor,
    DistanceSensorNum,
    MazeWidth,
    MaxSize,
    GoalSize,
    NullLogger,
> {
    let period = Time::from_seconds(0.001);

    let agent = {
        let estimator = {
            EstimatorBuilder::new()
                .left_encoder(IEncoder::new(Default::default()))
                .right_encoder(IEncoder::new(Default::default()))
                .imu(Imu::new(Default::default(), Default::default()))
                .period(period)
                .cut_off_frequency(Frequency::from_hertz(50.0))
                .initial_posture(Angle::from_degree(90.0))
                .initial_x(Distance::from_meters(0.045))
                .initial_y(Distance::from_meters(0.045))
                // .wheel_interval(Distance::from_meters(0.0335))
                .build()
        };

        let tracker = {
            let trans_controller = ControllerBuilder::new()
                .kp(0.9)
                .ki(0.05)
                .kd(0.01)
                .period(period)
                .model_gain(1.0)
                .model_time_constant(Time::from_seconds(0.3694))
                .build();

            let rot_controller = ControllerBuilder::new()
                .kp(0.2)
                .ki(0.2)
                .kd(0.0)
                .period(period)
                .model_gain(10.0)
                .model_time_constant(Time::from_seconds(0.1499))
                .build();

            TrackerBuilder::new()
                .right_motor(IMotor)
                .left_motor(IMotor)
                .period(period)
                .kx(40.0)
                .kdx(4.0)
                .ky(40.0)
                .kdy(4.0)
                .valid_control_lower_bound(Speed::from_meter_per_second(0.03))
                .translation_controller(trans_controller)
                .rotation_controller(rot_controller)
                .low_zeta(1.0)
                .low_b(1e-3)
                .fail_safe_distance(Distance::from_meters(0.05))
                .build()
        };

        let search_speed = Speed::from_meter_per_second(0.12);

        let trajectory_generator = TrajectoryGeneratorBuilder::new()
            .period(period)
            .max_speed(Speed::from_meter_per_second(2.0))
            .max_acceleration(Acceleration::from_meter_per_second_squared(0.7))
            .max_jerk(Jerk::from_meter_per_second_cubed(1.0))
            .search_speed(search_speed)
            .slalom_speed_ref(Speed::from_meter_per_second(0.27178875))
            .angular_speed_ref(AngularSpeed::from_radian_per_second(3.0 * PI))
            .angular_acceleration_ref(AngularAcceleration::from_radian_per_second_squared(
                36.0 * PI,
            ))
            .angular_jerk_ref(AngularJerk::from_radian_per_second_cubed(1200.0 * PI))
            .build();

        let obstacle_detector = {
            ObstacleDetector::new(arr![
                IDistanceSensor;
                IDistanceSensor::new(
                    Distance::from_meters(0.0305),
                    Pose {
                        x: Distance::from_meters(-0.0115),
                        y: Distance::from_meters(0.013),
                        theta: Angle::from_degree(90.0),
                    },
                ),
                IDistanceSensor::new(
                    Distance::from_meters(0.0305),
                    Pose {
                        x: Distance::from_meters(0.0115),
                        y: Distance::from_meters(0.013),
                        theta: Angle::from_degree(-90.0),
                    },
                ),
                IDistanceSensor::new(
                    Distance::from_meters(0.109),
                    Pose {
                        x: Distance::from_meters(0.0),
                        y: Distance::from_meters(0.023),
                        theta: Angle::from_degree(0.0),
                    },
                ),
            ])
        };
        Rc::new(Agent::new(
            obstacle_detector,
            estimator,
            tracker,
            trajectory_generator,
        ))
    };

    let maze = Rc::new(
        MazeBuilder::new()
            .costs(costs as fn(Pattern) -> u16)
            .wall_existence_probability_threshold(0.3)
            .build::<MazeWidth>(),
    );

    let solver = Rc::new(DefaultSolver::<MazeWidth, MaxSize, GoalSize>::new(
        NodeId::new(0, 0, AbsoluteDirection::North).unwrap(),
        arr![
            NodeId<MazeWidth>;
            NodeId::new(2, 0, AbsoluteDirection::South).unwrap(),
            NodeId::new(2, 0, AbsoluteDirection::West).unwrap(),
        ],
    ));

    let operator = DefaultSearchOperator::new(
        Pose::new(
            Distance::from_meters(0.045),
            Distance::from_meters(0.045),
            Angle::from_degree(90.0),
        ),
        SearchNodeId::new(0, 1, AbsoluteDirection::North).unwrap(),
        maze,
        agent,
        solver,
    );
    operator.init();
    operator
}
