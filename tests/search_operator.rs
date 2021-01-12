extern crate alloc;

#[macro_use]
extern crate typenum;

use core::f32::consts::PI;

use components::{
    data_types::{
        AbsoluteDirection, AngleState, LengthState, Pattern, Pose, SearchKind, SlalomDirection,
        SlalomKind, SlalomParameters, State,
    },
    impls::{
        slalom_parameters_map, slalom_parameters_map2, CommandConverter2, Commander,
        EstimatorBuilder, Maze, ObstacleDetector, PoseConverter, RotationControllerBuilder,
        RunNode, SearchAgent, SearchOperator, TrackerBuilder, TrajectoryGeneratorBuilder,
        TranslationControllerBuilder, WallConverter, WallManager,
    },
    utils::probability::Probability,
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
                    .map(|goal| RunNode::<Size>::new(goal.0, goal.1, goal.2, cost).unwrap())
                    .collect::<Vec<_>>();

                type MaxPathLength = op!(Size * Size);

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

                let wall_storage = WallManager::<Size>::with_str(existence_threshold, input_str);

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
                    let estimator = {
                        EstimatorBuilder::new()
                            .left_encoder(left_encoder)
                            .right_encoder(right_encoder)
                            .imu(imu)
                            .period(period)
                            .cut_off_frequency(Frequency::new::<hertz>(50.0))
                            .initial_posture(start_state.theta.x)
                            .initial_x(start_state.x.x)
                            .initial_y(start_state.y.x)
                            .wheel_interval(wheel_interval)
                            .build::<MathFake>()
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

                        TrackerBuilder::new()
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
                            .build::<MathFake>()
                    };

                    let search_velocity = Velocity::new::<meter_per_second>(0.12);

                    let trajectory_generator = TrajectoryGeneratorBuilder::new()
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
                        .run_slalom_velocity(Velocity::new::<meter_per_second>(1.0))
                        .front_offset(front_offset)
                        .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(90.0))
                        .spin_angular_acceleration(AngularAcceleration::new::<
                            degree_per_second_squared,
                        >(90.0))
                        .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(180.0))
                        .build::<MathFake, MaxPathLength>();

                    let obstacle_detector = ObstacleDetector::new(distance_sensors);
                    SearchAgent::new(obstacle_detector, estimator, tracker, trajectory_generator)
                };

                use AbsoluteDirection::*;

                let create_commander = |wall_storage| {
                    let pose_converter = PoseConverter::<Size, MathFake>::default();
                    let wall_converter = WallConverter::new(cost);
                    let maze = Maze::<_, _, _, MathFake>::new(
                        wall_storage,
                        pose_converter,
                        wall_converter,
                    );
                    let start = RunNode::<Size>::new(0, 0, North, cost).unwrap();
                    Commander::new(
                        start,
                        goals.clone(),
                        SearchKind::Init,
                        SearchKind::Final,
                        maze,
                    )
                };

                let commander = create_commander(WallManager::<Size>::new(existence_threshold));
                let expected_commander = create_commander(wall_storage);

                let operator = SearchOperator::new(
                    agent,
                    commander,
                    CommandConverter2::new(Length::new::<millimeter>(90.0), front_offset),
                );
                while operator.run().is_err() {
                    stepper.step();
                    operator.tick().expect("Fail safe should never invoke");
                }
                let (_, commander, _) = operator.consume();
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
    "+---+---+---+---+
|               |
+   +---+---+   +
|   |       |   |
+   +   +   +   +
|   |   |       |
+   +   +---+   +
|   |       |   |
+---+---+---+---+",
    vec![(2, 0, South), (2, 0, West)],
);

impl_search_operator_test!(
    search_operator_tests2: U16,
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
|                           |   |   |                           |
+   +---+---+---+   +---+   +---+---+   +---+   +---+---+---+   +
|   |           |   |                       |   |           |   |
+   +---+---+   +   +   +---+---+---+---+   +   +   +---+---+   +
|           |   |   |                       |   |   |           |
+   +---+   +---+   +---+---+---+---+---+---+   +---+   +---+   +
|   |   |   |   |           |       |           |   |   |   |   |
+   +---+   +---+---+---+---+---+---+   +---+   +---+   +---+   +
|                   |                       |                   |
+   +---+---+   +   +---+---+   +---+---+---+---+---+---+---+   +
|   |       |   |   |       |   |   |       |       |           |
+   +---+   +   +   +   +---+   +---+---+   +   +   +   +---+   +
|       |   |   |   |   |               |   |   |   |   |       |
+---+   +   +   +   +---+   +---+---+   +---+   +   +   +   +   +
|   |   |   |       |   |   |       |   |   |       |   |   |   |
+   +   +---+   +   +   +   +   +   +   +   +   +   +   +   +   +
|   |   |       |   |   |   |           |   |   |   |   |   |   |
+---+   +---+   +   +---+   +---+---+   +---+   +   +   +   +   +
|       |   |   |   |   |               |   |   |   |   |       |
+   +---+   +   +   +   +---+---+---+---+   +   +   +   +---+   +
|   |       |       |           |           |       |           |
+   +---+---+---+   +---+---+---+---+---+---+---+   +---+---+   +
|               |                               |               |
+   +---+---+   +   +---+   +---+---+   +---+   +---+---+---+   +
|               |                               |   |       |   |
+   +---+---+   +   +---+---+---+---+---+---+   +---+---+   +   +
|   |   |   |       |           |           |           |   |   |
+   +   +---+   +   +   +---+---+---+---+   +   +---+   +   +   +
|   |   |       |   |   |               |   |           |   |   |
+   +   +   +---+   +---+   +---+---+   +---+   +---+   +---+   +
|   |   |                                       |   |           |
+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",
    vec![
        (15, 14, SouthWest),
        (15, 14, SouthEast),
        (15, 16, NorthWest),
        (15, 16, NorthEast)
    ],
);
