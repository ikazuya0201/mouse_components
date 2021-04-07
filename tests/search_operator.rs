#[cfg(feature = "log_test")]
use std::{cell::RefCell, io::Write};

use components::{
    agents::TrackingAgent,
    command_converter::CommandConverter,
    commanders::SearchCommander,
    controllers::{RotationalControllerBuilder, TranslationalControllerBuilder},
    estimator::EstimatorBuilder,
    mazes::Maze,
    nodes::{Node, RunNode, SearchNode},
    obstacle_detector::ObstacleDetector,
    operators::TrackingOperator,
    pattern_converters::LinearPatternConverter,
    robot::Robot,
    tracker::TrackerBuilder,
    trajectory_generators::{DefaultSlalomParametersGenerator, SearchTrajectoryGeneratorBuilder},
    trajectory_managers::SearchTrajectoryManager,
    types::data::{AbsoluteDirection, AngleState, LengthState, Pose, RobotState, SearchKind},
    utils::probability::Probability,
    wall_detector::WallDetectorBuilder,
    wall_manager::WallManager,
};
use typenum::consts::*;
use uom::si::f32::{
    Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, ElectricPotential,
    Frequency, Jerk, Length, Time, Velocity,
};
use uom::si::{
    acceleration::meter_per_second_squared,
    angle::degree,
    angular_acceleration::degree_per_second_squared,
    angular_jerk::degree_per_second_cubed,
    angular_velocity::degree_per_second,
    electric_potential::volt,
    frequency::hertz,
    jerk::meter_per_second_cubed,
    length::{meter, millimeter},
    time::{millisecond, second},
    velocity::meter_per_second,
};
use utils::math::MathFake;
use utils::sensors::AgentSimulator;

macro_rules! impl_search_operator_test {
    ($name: ident: $size: ty, $input_str: expr, $goals: expr,) => {
        mod $name {
            use super::*;

            type Size = $size;

            fn _test_search_operator(front_offset: Length, distance_sensors_poses: Vec<Pose>) {
                use components::prelude::*;

                let input_str = $input_str;
                let goals = $goals
                    .into_iter()
                    .map(|goal| RunNode::<Size>::new(goal.0, goal.1, goal.2).unwrap())
                    .collect::<Vec<_>>();

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
                let existence_threshold = Probability::new(0.1).unwrap();
                let wheel_interval = Length::new::<millimeter>(33.5);

                let expected_wall_manager =
                    WallManager::<Size>::with_str(existence_threshold, input_str);
                let wall_manager = WallManager::<Size>::new(existence_threshold);

                let square_width = Length::new::<meter>(0.09);
                let wall_width = Length::new::<meter>(0.006);
                let ignore_radius_from_pillar = Length::new::<meter>(0.01);
                let ignore_length_from_wall = Length::new::<meter>(0.008);

                let simulator = AgentSimulator::new(
                    start_state.clone(),
                    period,
                    trans_model_gain,
                    trans_model_time_constant,
                    rot_model_gain,
                    rot_model_time_constant,
                    WallManager::<Size>::with_str(existence_threshold, input_str),
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
                ) = simulator.split(
                    wheel_interval,
                    ElectricPotential::new::<volt>(3.7),
                    square_width,
                    wall_width,
                    ignore_radius_from_pillar,
                    ignore_length_from_wall,
                );

                #[cfg(feature = "log_test")]
                let logs = RefCell::new(Vec::new());

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
                                .correction_weight(0.1)
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
                                .ki(0.2)
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
                                .kx(15.0)
                                .kdx(4.0)
                                .ky(15.0)
                                .kdy(4.0)
                                .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.05))
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
                                utils::logged_tracker::JsonLoggedTracker::new(tracker, &logs)
                            }
                        };

                        let wall_detector = {
                            let obstacle_detector =
                                ObstacleDetector::<_, MathFake>::new(distance_sensors);
                            WallDetectorBuilder::new()
                                .wall_manager(&wall_manager)
                                .obstacle_detector(obstacle_detector)
                                .build::<MathFake, Size>()
                                .unwrap()
                        };
                        Robot::new(estimator, tracker, wall_detector)
                    };

                    let search_velocity = Velocity::new::<meter_per_second>(0.12);

                    let trajectory_generator = SearchTrajectoryGeneratorBuilder::default()
                        .period(period)
                        .max_velocity(Velocity::new::<meter_per_second>(0.5))
                        .max_acceleration(Acceleration::new::<meter_per_second_squared>(20.0))
                        .max_jerk(Jerk::new::<meter_per_second_cubed>(40.0))
                        .search_velocity(search_velocity)
                        .parameters_generator(DefaultSlalomParametersGenerator::new(
                            square_width,
                            front_offset,
                        ))
                        .front_offset(front_offset)
                        .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(180.0))
                        .spin_angular_acceleration(AngularAcceleration::new::<
                            degree_per_second_squared,
                        >(1800.0))
                        .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(7200.0))
                        .build()
                        .expect("Should never panic");

                    let trajectory_manager = SearchTrajectoryManager::new(
                        trajectory_generator,
                        CommandConverter::new(Length::new::<millimeter>(90.0), front_offset),
                    );

                    TrackingAgent::new(trajectory_manager, robot)
                };

                use AbsoluteDirection::*;

                let create_commander = |wall_storage| {
                    let maze: Maze<_, _, SearchNode<Size>> =
                        Maze::new(wall_storage, LinearPatternConverter::default());
                    let start = RunNode::<Size>::new(0, 0, North).unwrap();
                    let current: Node<Size> = start.clone().into();
                    SearchCommander::new(
                        start,
                        &goals,
                        current,
                        SearchKind::Init,
                        SearchKind::Final,
                        maze,
                    )
                };

                let commander = create_commander(&wall_manager);
                let expected_commander = create_commander(&expected_wall_manager);

                #[cfg(feature = "log_test")]
                let output_log = || {
                    use std::ops::Deref;

                    writeln!(
                        std::io::stdout(),
                        "{}",
                        serde_json::to_string(logs.borrow().deref()).unwrap()
                    )
                    .unwrap();
                };

                let operator = TrackingOperator::new(commander, agent);
                while operator.run().is_err() {
                    stepper.step();
                    if let Err(err) = operator.tick() {
                        #[cfg(feature = "log_test")]
                        output_log();
                        panic!("Should never panic: {:?}", err);
                    }
                }

                let commander = create_commander(&wall_manager);
                assert_eq!(
                    commander.compute_shortest_path(),
                    expected_commander.compute_shortest_path()
                );

                #[cfg(feature = "log_test")]
                output_log();
            }

            #[ignore]
            #[test]
            fn test_search_operator() {
                _test_search_operator(
                    Default::default(),
                    vec![
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
                    ],
                );
            }

            #[ignore]
            #[test]
            fn test_search_operator_with_front_offset() {
                _test_search_operator(
                    Length::new::<millimeter>(10.0),
                    vec![
                        Pose {
                            x: Length::new::<millimeter>(0.0),
                            y: Length::new::<millimeter>(23.0),
                            theta: Angle::new::<degree>(0.0),
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
                    ],
                );
            }
        }
    };
}

impl_search_operator_test!(
    search_operator_tests1: U4,
    include_str!("../mazes/maze1.dat"),
    vec![(2, 0, South), (2, 0, West)],
);

impl_search_operator_test!(
    search_operator_tests2: U16,
    include_str!("../mazes/maze2.dat"),
    vec![
        (15, 14, SouthWest),
        (15, 14, SouthEast),
        (15, 16, NorthWest),
        (15, 16, NorthEast)
    ],
);

impl_search_operator_test!(
    search_operator_tests3: U4,
    include_str!("../mazes/maze3.dat"),
    vec![(2, 0, South), (2, 0, West)],
);
