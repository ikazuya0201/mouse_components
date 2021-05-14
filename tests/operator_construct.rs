extern crate alloc;

use alloc::rc::Rc;

use components::{
    command_converter::CommandConverter,
    defaults::{
        alias::{
            ReturnOperator, ReturnSetupOperator, RunOperator, RunSetupOperator, SearchOperator,
        },
        config::{ConfigBuilder, ConfigContainer},
        resource::{ResourceBuilder, ResourceContainer},
        state::{State, StateBuilder, StateContainer},
    },
    nodes::RunNode,
    pattern_converters::LinearPatternConverter,
    prelude::*,
    types::data::{AbsoluteDirection, ControlParameters, Pose, SearchKind},
    utils::probability::Probability,
    wall_manager::WallManager,
};
use uom::si::{
    acceleration::meter_per_second_squared,
    angle::degree,
    angular_acceleration::degree_per_second_squared,
    angular_jerk::degree_per_second_cubed,
    angular_velocity::degree_per_second,
    electric_potential::volt,
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, ElectricPotential,
        Frequency, Jerk, Length, Time, Velocity,
    },
    frequency::hertz,
    jerk::meter_per_second_cubed,
    length::{meter, millimeter},
    time::second,
    velocity::meter_per_second,
};
use utils::sensors::{AgentSimulator, DistanceSensor, Encoder, Motor, IMU};

macro_rules! impl_construct_and_deconstruct_test {
    ($name: ident: $operator: ident, $wall_manager: expr, $state: expr, $maze_width: literal,) => {
        #[test]
        fn $name() {
            use AbsoluteDirection::*;

            let existence_threshold = Probability::new(0.1).unwrap();

            let config = ConfigBuilder::new()
                .start(RunNode::<$maze_width>::new(0, 0, North).unwrap())
                .return_goal(RunNode::<$maze_width>::new(0, 0, South).unwrap())
                .goals(
                    vec![
                        RunNode::<$maze_width>::new(2, 0, South).unwrap(),
                        RunNode::<$maze_width>::new(2, 0, West).unwrap(),
                    ]
                    .into_iter()
                    .collect(),
                )
                .search_initial_route(SearchKind::Init)
                .search_final_route(SearchKind::Final)
                .pattern_converter(LinearPatternConverter::default())
                .estimator_cut_off_frequency(Frequency::new::<hertz>(50.0))
                .period(Time::new::<second>(0.001))
                .estimator_correction_weight(0.1)
                .translational_parameters(ControlParameters {
                    kp: 1.0,
                    ki: 0.05,
                    kd: 0.01,
                    model_k: 1.0,
                    model_t1: 0.3694,
                })
                .rotational_parameters(ControlParameters {
                    kp: 1.0,
                    ki: 0.2,
                    kd: 0.0,
                    model_k: 10.0,
                    model_t1: 0.1499,
                })
                .tracker_gain(40.0)
                .tracker_dgain(4.0)
                .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.03))
                .low_zeta(1.0)
                .low_b(1e-3)
                .fail_safe_distance(uom::si::f32::Length::new::<meter>(0.05))
                .search_velocity(Velocity::new::<meter_per_second>(0.12))
                .max_velocity(Velocity::new::<meter_per_second>(1.0))
                .max_acceleration(Acceleration::new::<meter_per_second_squared>(50.0))
                .max_jerk(Jerk::new::<meter_per_second_cubed>(100.0))
                .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(180.0))
                .spin_angular_acceleration(AngularAcceleration::new::<degree_per_second_squared>(
                    1800.0,
                ))
                .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(7200.0))
                .run_slalom_velocity(Velocity::new::<meter_per_second>(0.5))
                .slip_angle_const(Acceleration::new::<meter_per_second_squared>(100.0))
                .build()
                .unwrap();

            let distance_sensors_poses = vec![
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
            ];

            let wheel_interval = Length::new::<millimeter>(33.5);

            let simulator = AgentSimulator::new(
                $state.robot_state().clone(),
                *config.period(),
                config.translational_parameters().model_k,
                Time::new::<second>(config.translational_parameters().model_t1),
                config.rotational_parameters().model_k,
                Time::new::<second>(config.rotational_parameters().model_t1),
                WallManager::<$maze_width>::with_str(
                    existence_threshold,
                    include_str!("../mazes/maze1.dat"),
                ),
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
                *config.square_width(),
                *config.wall_width(),
                *config.ignore_radius_from_pillar(),
                *config.ignore_length_from_wall(),
            );

            let mut resource: ResourceContainer<_, _, _, _, _, _, $maze_width> =
                ResourceBuilder::new()
                    .left_encoder(left_encoder)
                    .right_encoder(right_encoder)
                    .imu(imu)
                    .left_motor(left_motor)
                    .right_motor(right_motor)
                    .wall_manager(Rc::new($wall_manager))
                    .distance_sensors(distance_sensors.into_iter().collect())
                    .build()
                    .unwrap()
                    .into();

            let config: ConfigContainer<$maze_width> = config.into();
            let state: StateContainer<$maze_width> = $state.into();
            let operator = <$operator<
                Encoder,
                Encoder,
                IMU,
                Motor,
                Motor,
                DistanceSensor<$maze_width>,
                $maze_width,
            >>::construct(&config, &state, &mut resource);

            // Execute operator.
            while operator.run().is_err() {
                stepper.step();
                operator.tick().expect("Should never panic");
            }

            let (mut state_builder, mut resource_builder): (
                StateBuilder<$maze_width>,
                ResourceBuilder<_, _, _, _, _, _, $maze_width>,
            ) = operator.deconstruct();
            state_builder.build().expect("Should never panic"); // Assert if all fields exist.
            resource_builder.build().expect("Should never panic");
        }
    };
}

fn new_state<const N: usize>(x: i8, y: i8, dir: AbsoluteDirection) -> State<N> {
    let node = RunNode::new(x, y, dir).unwrap();
    let command_converter = CommandConverter::default();
    let (pose, _) = command_converter.convert(&(node, ()));
    State::new(node, pose.into())
}

impl_construct_and_deconstruct_test! {
    test_search_operator_construct_and_deconstruct:
    SearchOperator,
    WallManager::<4>::new(Probability::new(0.1).unwrap()),
    new_state::<4>(0, 0, North),
    4,
}

impl_construct_and_deconstruct_test! {
    test_run_operator_construct_and_deconstruct:
    RunOperator,
    WallManager::<4>::with_str(Probability::new(0.1).unwrap(), include_str!("../mazes/maze1.dat")),
    new_state::<4>(0, 0, North),
    4,
}

impl_construct_and_deconstruct_test! {
    test_return_setup_operator_construct_and_deconstruct:
    ReturnSetupOperator,
    WallManager::<16>::with_str(Probability::new(0.1).unwrap(), include_str!("../mazes/maze2.dat")),
    new_state::<16>(0, 26, South),
    16,
}

impl_construct_and_deconstruct_test! {
    test_return_operator_construct_and_deconstruct:
    ReturnOperator,
    WallManager::<4>::with_str(Probability::new(0.1).unwrap(), include_str!("../mazes/maze1.dat")),
    new_state::<4>(2, 0, North),
    4,
}

impl_construct_and_deconstruct_test! {
    test_run_setup_operator_construct_and_deconstruct:
    RunSetupOperator,
    WallManager::<4>::with_str(Probability::new(0.1).unwrap(), include_str!("../mazes/maze1.dat")),
    new_state::<4>(0, 0, South),
    4,
}
