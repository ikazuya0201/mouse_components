extern crate std;

#[cfg(feature = "log_test")]
use std::io::Write;

use components::{
    agents::TrackingAgent,
    command_converter::CommandConverter,
    commanders::RunCommander,
    controllers::{RotationalControllerBuilder, TranslationalControllerBuilder},
    estimator::EstimatorBuilder,
    mazes::CheckedMaze,
    nodes::{RunNode, SearchNode},
    obstacle_detector::ObstacleDetector,
    operators::TrackingOperator,
    pattern_converters::LinearPatternConverter,
    prelude::*,
    robot::Robot,
    tracker::TrackerBuilder,
    trajectory_generators::{DefaultSlalomParametersGenerator, RunTrajectoryGeneratorBuilder},
    trajectory_managers::TrackingTrajectoryManager,
    types::data::{AbsoluteDirection, AngleState, LengthState, Pose, RobotState},
    utils::probability::Probability,
    wall_detector::WallDetectorBuilder,
    wall_manager::WallManager,
};
use typenum::consts::*;
use uom::si::f32::{
    Acceleration, Angle, ElectricPotential, Frequency, Jerk, Length, Time, Velocity,
};
use uom::si::{
    acceleration::meter_per_second_squared,
    angle::degree,
    electric_potential::volt,
    frequency::hertz,
    jerk::meter_per_second_cubed,
    length::{meter, millimeter},
    time::{millisecond, second},
    velocity::meter_per_second,
};
use utils::math::MathFake;
use utils::sensors::AgentSimulator;

macro_rules! impl_run_operator_test {
    ($name: ident < $size: ty > { $input: expr, }) => {
        #[test]
        fn $name() {
            let start_state = RobotState {
                x: LengthState {
                    x: Length::new::<millimeter>(45.0),
                    ..Default::default()
                },
                y: LengthState {
                    x: Length::new::<millimeter>(45.0),
                    ..Default::default()
                },
                theta: AngleState {
                    x: Angle::new::<degree>(90.0),
                    ..Default::default()
                },
            };
            let period = Time::new::<millisecond>(1.0);
            let trans_model_gain = 1.0;
            let trans_model_time_constant = Time::new::<second>(0.3694);
            let rot_model_gain = 10.0;
            let rot_model_time_constant = Time::new::<second>(0.1499);
            let distance_sensors_poses = vec![
                Pose {
                    x: Length::new::<millimeter>(0.0),
                    y: Length::new::<millimeter>(23.0),
                    theta: Angle::new::<degree>(0.0),
                },
                Pose {
                    x: Length::new::<millimeter>(8.0),
                    y: Length::new::<millimeter>(20.0),
                    theta: Angle::new::<degree>(-45.0),
                },
                Pose {
                    x: Length::new::<millimeter>(-8.0),
                    y: Length::new::<millimeter>(20.0),
                    theta: Angle::new::<degree>(45.0),
                },
                Pose {
                    x: Length::new::<millimeter>(-11.5),
                    y: Length::new::<millimeter>(13.0),
                    theta: Angle::new::<degree>(90.0),
                },
                Pose {
                    x: Length::new::<millimeter>(11.5),
                    y: Length::new::<millimeter>(13.0),
                    theta: Angle::new::<degree>(-90.0),
                },
            ];
            let existence_threshold = Probability::new(0.1).unwrap();
            let wheel_interval = Length::new::<millimeter>(33.5);

            let (input_str, goals) = $input;

            let wall_storage = WallManager::<$size>::with_str(existence_threshold, input_str);

            let simulator = AgentSimulator::new(
                start_state.clone(),
                period,
                trans_model_gain,
                trans_model_time_constant,
                rot_model_gain,
                rot_model_time_constant,
                WallManager::<$size>::with_str(existence_threshold, input_str),
                distance_sensors_poses,
            );

            let (
                stepper,
                _observer,
                right_encoder,
                left_encoder,
                imu,
                right_motor,
                left_motor,
                distance_sensors,
            ) = simulator.split(wheel_interval, ElectricPotential::new::<volt>(3.7));

            let agent = {
                let robot = {
                    let estimator = {
                        EstimatorBuilder::default()
                            .left_encoder(left_encoder)
                            .right_encoder(right_encoder)
                            .imu(imu)
                            .period(period)
                            .cut_off_frequency(Frequency::new::<hertz>(50.0))
                            .initial_state(start_state)
                            .wheel_interval(Some(wheel_interval))
                            .build()
                            .unwrap()
                    };

                    let tracker = {
                        let trans_controller = TranslationalControllerBuilder::new()
                            .kp(1.0)
                            .ki(0.05)
                            .kd(0.01)
                            .period(period)
                            .model_gain(trans_model_gain)
                            .model_time_constant(trans_model_time_constant)
                            .build()
                            .unwrap();

                        let rot_controller = RotationalControllerBuilder::new()
                            .kp(1.0)
                            .ki(0.05)
                            .kd(0.01)
                            .period(period)
                            .model_gain(rot_model_gain)
                            .model_time_constant(rot_model_time_constant)
                            .build()
                            .unwrap();

                        let tracker = TrackerBuilder::default()
                            .right_motor(right_motor)
                            .left_motor(left_motor)
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
                            .unwrap();

                        #[cfg(not(feature = "log_test"))]
                        {
                            tracker
                        }
                        #[cfg(feature = "log_test")]
                        {
                            utils::logged_tracker::JsonLoggedTracker::new(
                                std::io::stdout(),
                                tracker,
                            )
                        }
                    };

                    let wall_detector = {
                        let obstacle_detector =
                            ObstacleDetector::<_, MathFake>::new(distance_sensors);
                        WallDetectorBuilder::new()
                            .wall_manager(&wall_storage)
                            .obstacle_detector(obstacle_detector)
                            .build::<MathFake, _>()
                            .unwrap()
                    };
                    Robot::new(estimator, tracker, wall_detector)
                };
                let trajectory_manager = {
                    let trajectory_generator = RunTrajectoryGeneratorBuilder::default()
                        .period(period)
                        .max_velocity(Velocity::new::<meter_per_second>(1.0))
                        .max_acceleration(Acceleration::new::<meter_per_second_squared>(50.0))
                        .max_jerk(Jerk::new::<meter_per_second_cubed>(100.0))
                        .parameters_generator(DefaultSlalomParametersGenerator)
                        .run_slalom_velocity(Velocity::new::<meter_per_second>(0.5))
                        .build()
                        .expect("Should never panic");
                    TrackingTrajectoryManager::new(
                        trajectory_generator,
                        CommandConverter::default(),
                    )
                };
                TrackingAgent::new(trajectory_manager, robot)
            };

            use AbsoluteDirection::*;

            let goals = goals
                .into_iter()
                .map(|(x, y, dir)| RunNode::new(x, y, dir).unwrap())
                .collect::<Vec<_>>();

            let commander = {
                let maze = CheckedMaze::<_, _, SearchNode<$size>>::new(
                    &wall_storage,
                    LinearPatternConverter::default(),
                );
                let start = RunNode::new(0, 0, North).unwrap();
                RunCommander::new(start, &goals, maze)
            };

            #[cfg(feature = "log_test")]
            write!(std::io::stdout(), "[").unwrap();

            let operator = TrackingOperator::new(commander, agent);
            while operator.run().is_err() {
                stepper.step();
                operator.tick().expect("Should never panic");
            }

            #[cfg(feature = "log_test")]
            write!(std::io::stdout(), "]").unwrap();
        }
    };
}

impl_run_operator_test! {
    test_run_operator1<U4> {
        (
            include_str!("../mazes/maze1.dat"),
            vec![
                (2, 0, South),
                (2, 0, West),
            ],
        ),
    }
}

impl_run_operator_test! {
    test_run_operator2<U16> {
        (
            include_str!("../mazes/maze2.dat"),
            vec![
                (15, 14, SouthWest),
                (15, 14, SouthEast),
                (15, 16, NorthWest),
                (15, 16, NorthEast)
            ],
        ),
    }
}

impl_run_operator_test!(
    test_run_operator3<U4> {
        (
            include_str!("../mazes/maze3.dat"),
            vec![(2, 0, South), (2, 0, West)],
        ),
    }
);
