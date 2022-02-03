use std::rc::Rc;

use mousecore::{
    agents::TrackingAgent,
    command_converter::CommandConverter,
    commanders::SearchCommander,
    controllers::MultiSisoController,
    estimator::EstimatorBuilder,
    mazes::Maze,
    nodes::{Node, RunNode, SearchNode},
    operators::TrackingOperator,
    pattern_converters::DefaultPatternConverter,
    robot::Robot,
    solvers::dijkstra::DijkstraSolver,
    tracker::TrackerBuilder,
    trajectory_generators::{DefaultSlalomParametersGenerator, SearchTrajectoryGeneratorBuilder},
    trajectory_managers::SearchTrajectoryManager,
    types::data::{
        AbsoluteDirection, AngleState, ControlParameters, LengthState, Pose, Position, RobotState,
        SearchKind,
    },
    utils::probability::Probability,
    utils::random::Random,
    wall_detector::WallDetectorBuilder,
    wall_manager::WallManager,
};
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
use utils::sensors::AgentSimulator;

macro_rules! impl_search_operator_test {
    ($name: ident: $size: literal, $input_str: expr, $goal: expr,) => {
        mod $name {
            use super::*;

            const N: usize = $size;

            fn _test_search_operator(front_offset: Length, distance_sensors_poses: Vec<Pose>) {
                use mousecore::prelude::*;

                let input_str = $input_str;
                let goal = $goal;
                let goal = Position::new(goal.0, goal.1).unwrap();

                let start_state = RobotState {
                    x: LengthState {
                        x: Random {
                            mean: Length::new::<millimeter>(45.0),
                            standard_deviation: Default::default(),
                        },
                        ..Default::default()
                    },
                    y: LengthState {
                        x: Random {
                            mean: Length::new::<millimeter>(45.0),
                            standard_deviation: Default::default(),
                        },
                        ..Default::default()
                    },
                    theta: AngleState {
                        x: Angle::new::<degree>(90.0),
                        ..Default::default()
                    },
                };
                let period = Time::new::<millisecond>(1.0);
                let trans_model_k = 1.865;
                let trans_model_t1 = Time::new::<second>(0.4443);
                let rot_model_k = 82.39;
                let rot_model_t1 = Time::new::<second>(0.2855);
                let existence_threshold = Probability::new(0.1).unwrap();
                let wheel_interval = Length::new::<millimeter>(33.5);

                let expected_wall_manager =
                    WallManager::<N>::with_str(existence_threshold, input_str);
                let wall_manager = Rc::new(WallManager::<N>::new(existence_threshold));

                let square_width = Length::new::<meter>(0.09);
                let wall_width = Length::new::<meter>(0.006);
                let ignore_radius_from_pillar = Length::new::<meter>(0.01);
                let ignore_length_from_wall = Length::new::<meter>(0.008);

                let simulator = AgentSimulator::new(
                    start_state.clone(),
                    period,
                    trans_model_k,
                    trans_model_t1,
                    rot_model_k,
                    rot_model_t1,
                    WallManager::<N>::with_str(existence_threshold, input_str),
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
                                .slip_angle_const(Acceleration::new::<meter_per_second_squared>(
                                    100.0,
                                ))
                                .standard_deviation_delta(Length::new::<meter>(1e-6))
                                .build()
                                .unwrap()
                        };

                        let tracker = {
                            let controller = MultiSisoController::new(
                                left_motor,
                                right_motor,
                                ControlParameters {
                                    kp: 4.8497,
                                    ki: 29.5783,
                                    kd: 0.0,
                                    model_k: trans_model_k,
                                    model_t1: trans_model_t1.value,
                                },
                                ControlParameters {
                                    kp: 0.21134,
                                    ki: 2.9317,
                                    kd: 0.0,
                                    model_k: rot_model_k,
                                    model_t1: rot_model_t1.value,
                                },
                                period,
                                ElectricPotential::new::<volt>(10.0),
                            );

                            TrackerBuilder::new()
                                .period(period)
                                .gain(40.0)
                                .dgain(4.0)
                                .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.2))
                                .controller(controller)
                                .low_zeta(1.0)
                                .low_b(1.0)
                                .build()
                                .unwrap()
                        };

                        let wall_detector = {
                            WallDetectorBuilder::new()
                                .wall_manager(Rc::clone(&wall_manager))
                                .distance_sensors(distance_sensors.into_iter().collect())
                                .build::<N>()
                                .unwrap()
                        };
                        Robot::new(estimator, tracker, wall_detector)
                    };

                    let search_velocity = Velocity::new::<meter_per_second>(0.3);

                    let trajectory_generator = SearchTrajectoryGeneratorBuilder::default()
                        .period(period)
                        .max_acceleration(Acceleration::new::<meter_per_second_squared>(10.0))
                        .max_jerk(Jerk::new::<meter_per_second_cubed>(100.0))
                        .search_velocity(search_velocity)
                        .parameters_generator(DefaultSlalomParametersGenerator::new(
                            square_width,
                            front_offset,
                        ))
                        .front_offset(front_offset)
                        .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(1440.0))
                        .spin_angular_acceleration(AngularAcceleration::new::<
                            degree_per_second_squared,
                        >(14400.0))
                        .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(28800.0))
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
                    let maze: Maze<_, _, SearchNode<N>> =
                        Maze::new(wall_storage, DefaultPatternConverter::default());
                    let start = RunNode::<N>::new(0, 0, North).unwrap();
                    let current: Node<N> = start.clone().into();
                    SearchCommander::new(
                        start,
                        goal.clone(),
                        current,
                        SearchKind::Init,
                        SearchKind::Final,
                        maze,
                        DijkstraSolver,
                    )
                };

                let commander = create_commander(Rc::clone(&wall_manager));
                let expected_commander = create_commander(Rc::new(expected_wall_manager));

                let operator = TrackingOperator::new(commander, agent);
                while operator.run().is_err() {
                    stepper.step();
                    if let Err(err) = operator.tick() {
                        panic!("Should never panic: {:?}", err);
                    }
                }

                let commander = create_commander(Rc::clone(&wall_manager));
                assert_eq!(
                    commander.compute_shortest_path(),
                    expected_commander.compute_shortest_path()
                );
            }

            #[ignore]
            #[test]
            fn test_search_operator() {
                _test_search_operator(
                    Default::default(),
                    vec![
                        Pose {
                            x: Length::new::<millimeter>(23.0),
                            y: Length::new::<millimeter>(0.0),
                            theta: Angle::new::<degree>(0.0),
                        },
                        Pose {
                            x: Length::new::<millimeter>(20.0),
                            y: Length::new::<millimeter>(-8.0),
                            theta: Angle::new::<degree>(-45.0),
                        },
                        Pose {
                            x: Length::new::<millimeter>(20.0),
                            y: Length::new::<millimeter>(8.0),
                            theta: Angle::new::<degree>(45.0),
                        },
                        Pose {
                            x: Length::new::<millimeter>(13.0),
                            y: Length::new::<millimeter>(11.5),
                            theta: Angle::new::<degree>(90.0),
                        },
                        Pose {
                            x: Length::new::<millimeter>(13.0),
                            y: Length::new::<millimeter>(-11.5),
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
                            x: Length::new::<millimeter>(23.0),
                            y: Length::new::<millimeter>(0.0),
                            theta: Angle::new::<degree>(0.0),
                        },
                        Pose {
                            x: Length::new::<millimeter>(13.0),
                            y: Length::new::<millimeter>(11.5),
                            theta: Angle::new::<degree>(90.0),
                        },
                        Pose {
                            x: Length::new::<millimeter>(13.0),
                            y: Length::new::<millimeter>(-11.5),
                            theta: Angle::new::<degree>(-90.0),
                        },
                    ],
                );
            }
        }
    };
}

impl_search_operator_test!(
    search_operator_tests1: 4,
    include_str!("../mazes/maze1.dat"),
    (2, 0),
);

impl_search_operator_test!(
    search_operator_tests2: 16,
    include_str!("../mazes/maze2.dat"),
    (14, 14),
);

impl_search_operator_test!(
    search_operator_tests3: 4,
    include_str!("../mazes/maze3.dat"),
    (2, 0),
);

impl_search_operator_test!(
    search_operator_test_corner1: 16,
    include_str!("../mazes/search-corner1.dat"),
    (14, 14),
);
