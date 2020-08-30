#[macro_use]
extern crate typenum;

mod sensors {
    use core::marker::PhantomData;

    use components::{
        data_types::Pose,
        sensors::{DistanceSensor, Encoder, Motor, IMU},
        utils::sample::Sample,
    };
    use uom::si::f32::{Acceleration, AngularVelocity, ElectricPotential, Length};

    pub struct IDistanceSensor {
        distance: Length,
        pose: Pose,
    }

    impl IDistanceSensor {
        const SIGMA: Length = Length {
            dimension: PhantomData,
            units: PhantomData,
            value: 0.001,
        };

        pub fn new(distance: Length, pose: Pose) -> Self {
            Self { distance, pose }
        }
    }

    impl DistanceSensor for IDistanceSensor {
        type Error = ();

        fn pose(&self) -> Pose {
            self.pose
        }

        fn get_distance(&mut self) -> nb::Result<Sample<Length>, Self::Error> {
            Ok(Sample {
                mean: self.distance,
                standard_deviation: Self::SIGMA,
            })
        }
    }

    pub struct IEncoder {
        distance: Length,
    }

    impl IEncoder {
        pub fn new(distance: Length) -> Self {
            Self { distance }
        }
    }

    impl Encoder for IEncoder {
        type Error = ();

        fn get_relative_distance(&mut self) -> nb::Result<Length, Self::Error> {
            Ok(self.distance)
        }
    }

    pub struct IMotor;

    impl Motor for IMotor {
        fn apply(&mut self, _voltage: ElectricPotential) {}
    }

    pub struct Imu {
        accelerations: [Acceleration; 3],
        angular_velocitys: [AngularVelocity; 3],
    }

    impl Imu {
        pub fn new(
            accelerations: [Acceleration; 3],
            angular_velocitys: [AngularVelocity; 3],
        ) -> Self {
            Self {
                accelerations,
                angular_velocitys,
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

        fn get_angular_velocity_x(&mut self) -> nb::Result<AngularVelocity, Self::Error> {
            Ok(self.angular_velocitys[0])
        }
        fn get_angular_velocity_y(&mut self) -> nb::Result<AngularVelocity, Self::Error> {
            Ok(self.angular_velocitys[1])
        }
        fn get_angular_velocity_z(&mut self) -> nb::Result<AngularVelocity, Self::Error> {
            Ok(self.angular_velocitys[2])
        }
    }
}

use std::f32::consts::PI;
use std::rc::Rc;

use components::{
    data_types::{AbsoluteDirection, NodeId, Pattern, Pose, SearchNodeId},
    defaults::{DefaultSearchOperator, DefaultSolver},
    impls::{
        Agent, EstimatorBuilder, MazeBuilder, NullLogger, ObstacleDetector,
        RotationControllerBuilder, TrackerBuilder, TrajectoryGeneratorBuilder,
        TranslationControllerBuilder,
    },
    prelude::*,
};
use criterion::*;
use generic_array::arr;
use typenum::consts::*;
use uom::si::{
    acceleration::meter_per_second_squared,
    angle::degree,
    angular_acceleration::radian_per_second_squared,
    angular_jerk::radian_per_second_cubed,
    angular_velocity::radian_per_second,
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, Frequency, Jerk,
        Length, Time, Velocity,
    },
    frequency::hertz,
    jerk::meter_per_second_cubed,
    length::meter,
    time::second,
    velocity::meter_per_second,
};

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
    let period = Time::new::<second>(0.001);

    let agent = {
        let estimator = {
            EstimatorBuilder::new()
                .left_encoder(IEncoder::new(Default::default()))
                .right_encoder(IEncoder::new(Default::default()))
                .imu(Imu::new(Default::default(), Default::default()))
                .period(period)
                .cut_off_frequency(Frequency::new::<hertz>(50.0))
                .initial_posture(Angle::new::<degree>(90.0))
                .initial_x(Length::new::<meter>(0.045))
                .initial_y(Length::new::<meter>(0.045))
                // .wheel_interval(Length::new::<meter>(0.0335))
                .build()
        };

        let tracker = {
            let trans_controller = TranslationControllerBuilder::new()
                .kp(0.9)
                .ki(0.05)
                .kd(0.01)
                .period(period)
                .model_gain(1.0)
                .model_time_constant(Time::new::<second>(0.3694))
                .build();

            let rot_controller = RotationControllerBuilder::new()
                .kp(0.2)
                .ki(0.2)
                .kd(0.0)
                .period(period)
                .model_gain(10.0)
                .model_time_constant(Time::new::<second>(0.1499))
                .build();

            TrackerBuilder::new()
                .right_motor(IMotor)
                .left_motor(IMotor)
                .period(period)
                .kx(40.0)
                .kdx(4.0)
                .ky(40.0)
                .kdy(4.0)
                .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.03))
                .translation_controller(trans_controller)
                .rotation_controller(rot_controller)
                .low_zeta(1.0)
                .low_b(1e-3)
                .fail_safe_distance(Length::new::<meter>(0.05))
                .build()
        };

        let search_velocity = Velocity::new::<meter_per_second>(0.12);

        let trajectory_generator = TrajectoryGeneratorBuilder::new()
            .period(period)
            .max_velocity(Velocity::new::<meter_per_second>(2.0))
            .max_acceleration(Acceleration::new::<meter_per_second_squared>(0.7))
            .max_jerk(Jerk::new::<meter_per_second_cubed>(1.0))
            .search_velocity(search_velocity)
            .slalom_velocity_ref(Velocity::new::<meter_per_second>(0.27178875))
            .angular_velocity_ref(AngularVelocity::new::<radian_per_second>(3.0 * PI))
            .angular_acceleration_ref(AngularAcceleration::new::<radian_per_second_squared>(
                36.0 * PI,
            ))
            .angular_jerk_ref(AngularJerk::new::<radian_per_second_cubed>(1200.0 * PI))
            .build();

        let obstacle_detector = {
            ObstacleDetector::new(arr![
                IDistanceSensor;
                IDistanceSensor::new(
                    Length::new::<meter>(0.0305),
                    Pose {
                        x: Length::new::<meter>(-0.0115),
                        y: Length::new::<meter>(0.013),
                        theta: Angle::new::<degree>(90.0),
                    },
                ),
                IDistanceSensor::new(
                    Length::new::<meter>(0.0305),
                    Pose {
                        x: Length::new::<meter>(0.0115),
                        y: Length::new::<meter>(0.013),
                        theta: Angle::new::<degree>(-90.0),
                    },
                ),
                IDistanceSensor::new(
                    Length::new::<meter>(0.109),
                    Pose {
                        x: Length::new::<meter>(0.0),
                        y: Length::new::<meter>(0.023),
                        theta: Angle::new::<degree>(0.0),
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
            Length::new::<meter>(0.045),
            Length::new::<meter>(0.045),
            Angle::new::<degree>(90.0),
        ),
        SearchNodeId::new(0, 1, AbsoluteDirection::North).unwrap(),
        maze,
        agent,
        solver,
    );
    operator.init();
    operator
}
