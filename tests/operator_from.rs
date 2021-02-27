use components::defaults::{ReturnOperator, ReturnSetupOperator, RunOperator, SearchOperator};

macro_rules! impl_operator_from_test {
    ($name: ident: { $operator: ident, $wall_manager: expr, }) => {
        impl_operator_from_test! {
            $name: {
                $operator,
                $wall_manager,
                RunNode::<Size>::new(0, 0, North).unwrap(),
            }
        }
    };

    ($name: ident: { $operator: ident, $wall_manager: expr, $current_node: expr, }) => {
        #[test]
        fn $name() {
            use components::{
                config::{Config, ConfigBuilder},
                nodes::RunNode,
                obstacle_detector::ObstacleDetector,
                state::State,
                types::data::{
                    AbsoluteDirection, AngleState, LengthState, Pattern, Pose, RobotState,
                    SearchKind,
                },
                utils::probability::Probability,
                wall_manager::WallManager,
            };
            use typenum::consts::*;
            use uom::si::f32::{
                Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, Jerk,
                Length, Time, Velocity,
            };
            use uom::si::{
                acceleration::meter_per_second_squared,
                angle::degree,
                angular_acceleration::degree_per_second_squared,
                angular_jerk::degree_per_second_cubed,
                angular_velocity::degree_per_second,
                jerk::meter_per_second_cubed,
                length::{meter, millimeter},
                time::{millisecond, second},
                velocity::meter_per_second,
            };
            use utils::math::MathFake;
            use utils::sensors::{AgentSimulator, DistanceSensor, Encoder, Motor, IMU};

            use AbsoluteDirection::*;
            type Size = U4;

            let input_str = include_str!("../mazes/maze1.dat");
            let start = RunNode::<Size>::new(0, 0, North).unwrap();
            let return_goal = RunNode::<Size>::new(0, 0, South).unwrap();
            let goals = vec![(2, 0, South), (2, 0, West)]
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

            let wall_manager = $wall_manager;

            let distance_sensors_poses = vec![
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
            ];

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
                _,
                _observer,
                right_encoder,
                left_encoder,
                imu,
                right_motor,
                left_motor,
                distance_sensors,
            ) = simulator.split(wheel_interval);

            let obstacle_detector = ObstacleDetector::<_, MathFake>::new(distance_sensors);

            let resource = (
                &wall_manager,
                (
                    (left_encoder, right_encoder, imu),
                    (left_motor, right_motor),
                    (&wall_manager, obstacle_detector),
                ),
            );

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

            let config = ConfigBuilder::new()
                .start(start)
                .return_goal(return_goal)
                .goals(&goals)
                .search_initial_route(SearchKind::Init)
                .search_final_route(SearchKind::Final)
                .cost_fn(cost)
                .estimator_trans_velocity_alpha(0.1)
                .period(Time::new::<second>(0.001))
                .estimator_correction_weight(0.1)
                .translational_kp(0.9)
                .translational_ki(0.05)
                .translational_kd(0.01)
                .translational_model_gain(1.0)
                .translational_model_time_constant(Time::new::<second>(0.001))
                .rotational_kp(0.2)
                .rotational_ki(0.2)
                .rotational_kd(0.0)
                .rotational_model_gain(1.0)
                .rotational_model_time_constant(Time::new::<second>(0.001))
                .tracker_kx(15.0)
                .tracker_kdx(4.0)
                .tracker_ky(15.0)
                .tracker_kdy(4.0)
                .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.03))
                .low_zeta(1.0)
                .low_b(1e-3)
                .fail_safe_distance(uom::si::f32::Length::new::<meter>(0.05))
                .search_velocity(Velocity::new::<meter_per_second>(0.12))
                .max_velocity(Velocity::new::<meter_per_second>(2.0))
                .max_acceleration(Acceleration::new::<meter_per_second_squared>(0.7))
                .max_jerk(Jerk::new::<meter_per_second_cubed>(1.0))
                .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(180.0))
                .spin_angular_acceleration(AngularAcceleration::new::<degree_per_second_squared>(
                    1800.0,
                ))
                .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(7200.0))
                .run_slalom_velocity(Velocity::new::<meter_per_second>(1.0))
                .build()
                .unwrap();

            let state = State::new($current_node, start_state);

            //verify initialization
            let operator = <$operator<
                Encoder,
                Encoder,
                IMU,
                Motor,
                Motor,
                DistanceSensor<Size>,
                MathFake,
                Size,
            > as From<(
                _,
                &Config<RunNode<Size>, SearchKind, Pattern, u16>,
                &State<RunNode<Size>>,
            )>>::from((resource, &config, &state));

            use components::prelude::*;
            //verify execution (ignore result)
            let _ = operator.run();
            let _ = operator.tick();
        }
    };
}

impl_operator_from_test! {
    test_search_operator_from: {
        SearchOperator,
        WallManager::<Size>::new(Probability::new(0.1).unwrap()),
    }
}

impl_operator_from_test! {
    test_run_operator_from: {
        RunOperator,
        WallManager::<Size>::with_str(
            Probability::new(0.1).unwrap(),
            include_str!("../mazes/maze1.dat"),
        ),
    }
}

impl_operator_from_test! {
    test_return_operator_from: {
        ReturnOperator,
        WallManager::<Size>::with_str(
            Probability::new(0.1).unwrap(),
            include_str!("../mazes/maze1.dat"),
        ),
        RunNode::<Size>::new(4, 0, West).unwrap(),
    }
}

impl_operator_from_test! {
    test_return_setup_operator_from: {
        ReturnSetupOperator,
        WallManager::<Size>::with_str(
            Probability::new(0.1).unwrap(),
            include_str!("../mazes/maze1.dat"),
        ),
        RunNode::<Size>::new(4, 0, East).unwrap(),
    }
}
