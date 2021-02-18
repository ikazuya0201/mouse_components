use core::f32::consts::PI;

use components::{
    agents::SearchAgent,
    command_converter::CommandConverter,
    controllers::{RotationControllerBuilder, TranslationControllerBuilder},
    defaults,
    estimator::EstimatorBuilder,
    mazes::Maze,
    nodes::{RunNode, SearchNode},
    obstacle_detector::ObstacleDetector,
    operators::SearchOperator,
    pose_converter::PoseConverter,
    robot::Robot,
    tracker::TrackerBuilder,
    trajectory_generators::{
        slalom_parameters_map, slalom_parameters_map2, SearchTrajectoryGeneratorBuilder,
    },
    trajectory_managers::SearchTrajectoryManager,
    types::data::{
        AbsoluteDirection, AngleState, LengthState, Pattern, Pose, SearchKind, SlalomDirection,
        SlalomKind, SlalomParameters, State,
    },
    utils::probability::Probability,
    wall_detector::WallDetector,
    wall_manager::WallManager,
};
use typenum::consts::*;
use uom::si::f32::{
    Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, Frequency, Jerk,
    Length, Time, Velocity,
};
use uom::si::{
    acceleration::meter_per_second_squared,
    angle::degree,
    angular_acceleration::{degree_per_second_squared, radian_per_second_squared},
    angular_jerk::{degree_per_second_cubed, radian_per_second_cubed},
    angular_velocity::{degree_per_second, radian_per_second},
    frequency::hertz,
    jerk::meter_per_second_cubed,
    length::{meter, millimeter},
    time::{millisecond, second},
    velocity::meter_per_second,
};
use utils::math::MathFake;
use utils::sensors::AgentSimulator;

fn cost(pattern: Pattern) -> u16 {
    use Pattern::*;

    match pattern {
        Straight(x) => 10 * x,
        StraightDiagonal(x) => 7 * x,
        Search90 => 8,
        FastRun45 => 12,
        FastRun90 => 15,
        FastRun135 => 20,
        FastRun180 => 25,
        FastRunDiagonal90 => 15,
        SpinBack => 15,
    }
}

macro_rules! impl_search_operator_test {
    ($name: ident: $size: ty, $input_str: expr, $goals: expr,) => {
        mod $name {
            use super::*;

            type Size = $size;

            fn _test_search_operator(
                slalom_parameters_map: fn(SlalomKind, SlalomDirection) -> SlalomParameters,
                front_offset: Length,
                distance_sensors_poses: Vec<Pose>,
            ) {
                use components::prelude::*;

                let input_str = $input_str;
                let goals = $goals
                    .into_iter()
                    .map(|goal| RunNode::<Size>::new(goal.0, goal.1, goal.2).unwrap())
                    .collect::<Vec<_>>();

                let start_state = State {
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
                ) = simulator.split(wheel_interval);

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
                                .wheel_interval(wheel_interval)
                                .correction_weight(0.1)
                                .build()
                                .unwrap()
                        };

                        let tracker = {
                            let trans_controller = TranslationControllerBuilder::new()
                                .kp(0.9)
                                .ki(0.05)
                                .kd(0.01)
                                .period(period)
                                .model_gain(trans_model_gain)
                                .model_time_constant(trans_model_time_constant)
                                .build();

                            let rot_controller = RotationControllerBuilder::new()
                                .kp(0.2)
                                .ki(0.2)
                                .kd(0.0)
                                .period(period)
                                .model_gain(rot_model_gain)
                                .model_time_constant(rot_model_time_constant)
                                .build();

                            TrackerBuilder::default()
                                .right_motor(right_motor)
                                .left_motor(left_motor)
                                .period(period)
                                .kx(15.0)
                                .kdx(4.0)
                                .ky(15.0)
                                .kdy(4.0)
                                .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.03))
                                .translation_controller(trans_controller)
                                .rotation_controller(rot_controller)
                                .low_zeta(1.0)
                                .low_b(1e-3)
                                .fail_safe_distance(Length::new::<meter>(0.05))
                                .build()
                                .unwrap()
                        };

                        let wall_detector = {
                            let obstacle_detector =
                                ObstacleDetector::<_, MathFake>::new(distance_sensors);
                            let pose_converter = PoseConverter::<Size, MathFake>::default();
                            WallDetector::<_, _, _, MathFake>::new(
                                &wall_manager,
                                obstacle_detector,
                                pose_converter,
                            )
                        };
                        Robot::new(estimator, tracker, wall_detector)
                    };

                    let search_velocity = Velocity::new::<meter_per_second>(0.12);

                    let trajectory_generator = SearchTrajectoryGeneratorBuilder::default()
                        .period(period)
                        .max_velocity(Velocity::new::<meter_per_second>(2.0))
                        .max_acceleration(Acceleration::new::<meter_per_second_squared>(0.7))
                        .max_jerk(Jerk::new::<meter_per_second_cubed>(1.0))
                        .search_velocity(search_velocity)
                        .slalom_parameters_map(slalom_parameters_map)
                        .angular_velocity_ref(AngularVelocity::new::<radian_per_second>(3.0 * PI))
                        .angular_acceleration_ref(AngularAcceleration::new::<
                            radian_per_second_squared,
                        >(36.0 * PI))
                        .angular_jerk_ref(AngularJerk::new::<radian_per_second_cubed>(1200.0 * PI))
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

                    SearchAgent::new(trajectory_manager, robot)
                };

                use AbsoluteDirection::*;

                let create_commander = |wall_storage| {
                    let maze: Maze<_, _, _, SearchNode<Size>> = Maze::new(wall_storage, cost);
                    let start = RunNode::<Size>::new(0, 0, North).unwrap();
                    defaults::SearchCommander::new(
                        start.clone(),
                        &goals,
                        start.into(),
                        SearchKind::Init,
                        SearchKind::Final,
                        maze,
                    )
                };

                let commander = create_commander(&wall_manager);
                let expected_commander = create_commander(&expected_wall_manager);

                let operator = SearchOperator::new(commander, agent);
                while operator.run().is_err() {
                    stepper.step();
                    operator.tick().expect("Fail safe should never invoke");
                }
                let commander = create_commander(&wall_manager);
                assert_eq!(
                    commander.compute_shortest_path(),
                    expected_commander.compute_shortest_path()
                );
            }

            #[ignore]
            #[test]
            fn test_search_operator() {
                _test_search_operator(
                    slalom_parameters_map,
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
                    slalom_parameters_map2,
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
