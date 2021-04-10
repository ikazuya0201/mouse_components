extern crate alloc;

use alloc::rc::Rc;

use components::{
    command_converter::CommandConverter,
    defaults::{
        alias::{
            ReturnOperator, ReturnSetupOperator, RunOperator, RunSetupOperator, SearchOperator,
        },
        config::{ConfigBuilder, ConfigContainer},
        resource::ResourceBuilder,
        state::{State, StateBuilder, StateContainer},
    },
    nodes::RunNode,
    pattern_converters::LinearPatternConverter,
    prelude::*,
    types::data::{AbsoluteDirection, Pose, SearchKind},
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
use utils::{
    math::MathFake,
    sensors::{AgentSimulator, DistanceSensor, Encoder, Motor, IMU},
};

const N: usize = 4;

macro_rules! impl_construct_and_deconstruct_test {
    ($name: ident: $operator: ident, $wall_manager: expr, $state: expr,) => {
        #[test]
        fn $name() {
            use AbsoluteDirection::*;

            let existence_threshold = Probability::new(0.1).unwrap();

            let config = ConfigBuilder::new()
                .start(RunNode::<N>::new(0, 0, North).unwrap())
                .return_goal(RunNode::<N>::new(0, 0, South).unwrap())
                .goals(
                    vec![
                        RunNode::<N>::new(2, 0, South).unwrap(),
                        RunNode::<N>::new(2, 0, West).unwrap(),
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
                .translational_kp(1.0)
                .translational_ki(0.05)
                .translational_kd(0.01)
                .translational_model_gain(1.0)
                .translational_model_time_constant(Time::new::<second>(0.3694))
                .rotational_kp(1.0)
                .rotational_ki(0.2)
                .rotational_kd(0.0)
                .rotational_model_gain(10.0)
                .rotational_model_time_constant(Time::new::<second>(0.1499))
                .tracker_kx(40.0)
                .tracker_kdx(4.0)
                .tracker_ky(40.0)
                .tracker_kdy(4.0)
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
                .build()
                .unwrap();

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

            let wheel_interval = Length::new::<millimeter>(33.5);

            let simulator = AgentSimulator::new(
                $state.robot_state().clone(),
                *config.period(),
                *config.translational_model_gain(),
                *config.translational_model_time_constant(),
                *config.rotational_model_gain(),
                *config.rotational_model_time_constant(),
                WallManager::<N>::with_str(existence_threshold, include_str!("../mazes/maze1.dat")),
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

            let resource = ResourceBuilder::new()
                .left_encoder(left_encoder)
                .right_encoder(right_encoder)
                .imu(imu)
                .left_motor(left_motor)
                .right_motor(right_motor)
                .wall_manager(Rc::new($wall_manager))
                .distance_sensors(distance_sensors.into_iter().collect())
                .build()
                .unwrap();

            let config: ConfigContainer<N> = config.into();
            let state: StateContainer<N> = $state.into();
            let (operator, resource) = <$operator<
                Encoder,
                Encoder,
                IMU,
                Motor,
                Motor,
                DistanceSensor<N>,
                MathFake,
                N,
            >>::construct(&config, &state, resource);

            // Execute operator.
            while operator.run().is_err() {
                stepper.step();
                operator.tick().expect("Should never panic");
            }

            let (mut state_builder, dec_resource): (StateBuilder<N>, _) = operator.deconstruct();
            let resource = resource.merge(dec_resource);
            state_builder.build().expect("Should never panic"); // Assert if all fields exist.

            // Assert if all fields exist in resource.
            assert!(resource.left_encoder.is_some());
            assert!(resource.right_encoder.is_some());
            assert!(resource.imu.is_some());
            assert!(resource.left_motor.is_some());
            assert!(resource.right_motor.is_some());
            assert!(resource.distance_sensors.is_some());
            assert!(resource.wall_manager.is_some());
        }
    };
}

fn new_state(x: i8, y: i8, dir: AbsoluteDirection) -> State<N> {
    let node = RunNode::<N>::new(x, y, dir).unwrap();
    let command_converter = CommandConverter::default();
    let (pose, _) = command_converter.convert(&(node, ()));
    State::new(node, pose.into())
}

impl_construct_and_deconstruct_test! {
    test_search_operator_construct_and_deconstruct:
    SearchOperator,
    WallManager::<N>::new(Probability::new(0.1).unwrap()),
    new_state(0, 0, North),
}

impl_construct_and_deconstruct_test! {
    test_run_operator_construct_and_deconstruct:
    RunOperator,
    WallManager::<N>::with_str(Probability::new(0.1).unwrap(), include_str!("../mazes/maze1.dat")),
    new_state(0, 0, North),
}

impl_construct_and_deconstruct_test! {
    test_return_setup_operator_construct_and_deconstruct:
    ReturnSetupOperator,
    WallManager::<N>::with_str(Probability::new(0.1).unwrap(), include_str!("../mazes/maze1.dat")),
    new_state(2, 0, West),
}

impl_construct_and_deconstruct_test! {
    test_return_operator_construct_and_deconstruct:
    ReturnOperator,
    WallManager::<N>::with_str(Probability::new(0.1).unwrap(), include_str!("../mazes/maze1.dat")),
    new_state(2, 0, North),
}

impl_construct_and_deconstruct_test! {
    test_run_setup_operator_construct_and_deconstruct:
    RunSetupOperator,
    WallManager::<N>::with_str(Probability::new(0.1).unwrap(), include_str!("../mazes/maze1.dat")),
    new_state(0, 0, South),
}
