use core::ops::Mul;

use generic_array::ArrayLength;
use spin::Mutex;
use typenum::{consts::*, PowerOfTwo, Unsigned};

use crate::command_converter::CommandConverter;
use crate::commanders::CostNode;
use crate::controllers::{RotationalControllerBuilder, TranslationalControllerBuilder};
use crate::defaults::aliases::{Robot, RunAgent, RunCommander, SearchAgent, SearchCommander};
use crate::defaults::{config::Config, resource::Resource, state::State};
use crate::estimator::{Estimator, EstimatorBuilder};
use crate::mazes::{CheckedMaze, Maze, WallNode};
use crate::nodes::{Pattern, RunNode};
use crate::obstacle_detector::ObstacleDetector;
use crate::obstacle_detector::Vec;
use crate::sensors::{Encoder, Motor, IMU};
use crate::tracker::{Tracker, TrackerBuilder};
use crate::trajectory_generators::{
    RunKind, RunTrajectoryGenerator, RunTrajectoryGeneratorBuilder, SearchTrajectoryGenerator,
    SearchTrajectoryGeneratorBuilder,
};
use crate::trajectory_managers::{SearchTrajectoryManager, TrackingTrajectoryManager};
use crate::utils::probability::Probability;
use crate::wall_detector::{WallDetector, WallDetectorBuilder};
use crate::wall_manager::{Wall, WallManager};

pub fn init_estimator<'a, LeftEncoder, RightEncoder, Imu, Math, Size>(
    config: &'a Config<'a, Size>,
    state: &State<Size>,
    left_encoder: LeftEncoder,
    right_encoder: RightEncoder,
    imu: Imu,
) -> Estimator<LeftEncoder, RightEncoder, Imu, Math>
where
    Math: crate::utils::math::Math,
    LeftEncoder: Encoder,
    RightEncoder: Encoder,
    Imu: IMU,
{
    EstimatorBuilder::new()
        .left_encoder(left_encoder)
        .right_encoder(right_encoder)
        .imu(imu)
        .period(*config.period())
        .initial_state(state.robot_state().clone())
        .wheel_interval(*config.wheel_interval())
        .correction_weight(*config.estimator_correction_weight())
        .cut_off_frequency(*config.estimator_cut_off_frequency())
        .build()
        .expect("Should never panic")
}

pub fn init_tracker<'a, LeftMotor, RightMotor, Math, Size>(
    config: &'a Config<'a, Size>,
    left_motor: LeftMotor,
    right_motor: RightMotor,
) -> Tracker<LeftMotor, RightMotor, Math>
where
    Math: crate::utils::math::Math,
    RightMotor: Motor,
    LeftMotor: Motor,
{
    let translational_controller = TranslationalControllerBuilder::new()
        .period(*config.period())
        .kp(*config.translational_kp())
        .ki(*config.translational_ki())
        .kd(*config.translational_kd())
        .model_gain(*config.translational_model_gain())
        .model_time_constant(*config.translational_model_time_constant())
        .build()
        .expect("Should never panic");

    let rotational_controller = RotationalControllerBuilder::new()
        .period(*config.period())
        .kp(*config.rotational_kp())
        .ki(*config.rotational_ki())
        .kd(*config.rotational_kd())
        .model_gain(*config.rotational_model_gain())
        .model_time_constant(*config.rotational_model_time_constant())
        .build()
        .expect("Should never panic");

    TrackerBuilder::new()
        .period(*config.period())
        .kx(*config.kx())
        .kdx(*config.kdx())
        .ky(*config.ky())
        .kdy(*config.kdy())
        .low_b(*config.low_b())
        .low_zeta(*config.low_zeta())
        .fail_safe_distance(*config.fail_safe_distance())
        .valid_control_lower_bound(*config.valid_control_lower_bound())
        .translation_controller(translational_controller)
        .rotation_controller(rotational_controller)
        .right_motor(right_motor)
        .left_motor(left_motor)
        .build()
        .expect("Should never panic")
}

pub fn init_wall_detector<'a, DistanceSensor, Math, Size>(
    config: &Config<'a, Size>,
    distance_sensors: Vec<DistanceSensor>,
    wall_manager: &'a WallManager<Size>,
) -> WallDetector<'a, WallManager<Size>, ObstacleDetector<DistanceSensor, Math>, Math, Size>
where
    Math: crate::utils::math::Math,
    Size: Mul<Size>,
    Size::Output: Mul<U2>,
    <Size::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
{
    let obstacle_detector = ObstacleDetector::new(distance_sensors);
    WallDetectorBuilder::new()
        .wall_manager(wall_manager)
        .obstacle_detector(obstacle_detector)
        .wall_width(*config.wall_width())
        .square_width(*config.square_width())
        .ignore_length_from_wall(*config.ignore_length_from_wall())
        .ignore_radius_from_pillar(*config.ignore_radius_from_pillar())
        .build()
        .expect("Should never panic")
}

pub fn init_robot<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    Size,
>(
    config: &'a Config<'a, Size>,
    state: &State<Size>,
    resource: Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor>,
    wall_manager: &'a WallManager<Size>,
) -> Robot<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, Math, Size>
where
    Math: crate::utils::math::Math,
    LeftEncoder: Encoder,
    RightEncoder: Encoder,
    Imu: IMU,
    LeftMotor: Motor,
    RightMotor: Motor,
    Size: Mul<Size>,
    Size::Output: Mul<U2>,
    <Size::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
{
    let Resource {
        left_encoder,
        right_encoder,
        imu,
        left_motor,
        right_motor,
        distance_sensors,
    } = resource;
    Robot::new(
        init_estimator(config, state, left_encoder, right_encoder, imu),
        init_tracker(config, left_motor, right_motor),
        init_wall_detector(config, distance_sensors, wall_manager),
    )
}

pub fn init_search_trajectory_generator<'a, Math, Size>(
    config: &Config<'a, Size>,
) -> SearchTrajectoryGenerator<Math>
where
    Math: crate::utils::math::Math,
{
    SearchTrajectoryGeneratorBuilder::new()
        .period(*config.period())
        .max_jerk(*config.max_jerk())
        .max_velocity(*config.max_velocity())
        .max_acceleration(*config.max_acceleration())
        .front_offset(*config.front_offset())
        .search_velocity(*config.search_velocity())
        .angular_jerk_ref(*config.slalom_angular_jerk_ref())
        .angular_velocity_ref(*config.slalom_angular_velocity_ref())
        .angular_acceleration_ref(*config.slalom_angular_acceleration_ref())
        .slalom_parameters_map(*config.slalom_parameters_map())
        .spin_angular_jerk(*config.spin_angular_jerk())
        .spin_angular_velocity(*config.spin_angular_velocity())
        .spin_angular_acceleration(*config.spin_angular_acceleration())
        .build()
        .expect("Should never panic")
}

pub fn init_run_trajectory_generator<'a, Math, Size>(
    config: &Config<'a, Size>,
) -> RunTrajectoryGenerator<Math>
where
    Math: crate::utils::math::Math,
{
    RunTrajectoryGeneratorBuilder::new()
        .period(*config.period())
        .slalom_parameters_map(*config.slalom_parameters_map())
        .angular_acceleration_ref(*config.slalom_angular_acceleration_ref())
        .angular_velocity_ref(*config.slalom_angular_velocity_ref())
        .angular_jerk_ref(*config.slalom_angular_jerk_ref())
        .max_acceleration(*config.max_acceleration())
        .max_velocity(*config.max_velocity())
        .max_jerk(*config.max_jerk())
        .run_slalom_velocity(*config.run_slalom_velocity())
        .build()
        .expect("Should never panic")
}

pub fn init_search_commander<'a, Size>(
    config: &Config<'a, Size>,
    state: &State<Size>,
    wall_manager: &'a WallManager<Size>,
) -> SearchCommander<'a, Size>
where
    Size: Mul<Size> + Clone,
    Size::Output: Mul<U2>,
    <Size::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
{
    let maze = Maze::new(wall_manager, *config.cost_fn());
    SearchCommander::new(
        config.start().clone(),
        config.goals(),
        state.current_node().clone(),
        *config.search_initial_route(),
        *config.search_final_route(),
        maze,
    )
}

pub fn init_run_commander<'a, Size>(
    config: &Config<'a, Size>,
    state: &State<Size>,
    wall_manager: &'a WallManager<Size>,
) -> RunCommander<'a, Size>
where
    Size: Mul<Size> + Mul<U2> + Mul<U4> + Clone + PartialEq + PowerOfTwo + Unsigned,
    <Size as Mul<Size>>::Output: Mul<U2> + Mul<U16>,
    <Size as Mul<U4>>::Output: ArrayLength<(RunNode<Size>, u16)>
        + ArrayLength<WallNode<Wall<Size>, (RunNode<Size>, Pattern)>>,
    <<Size as Mul<Size>>::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
    <<Size as Mul<Size>>::Output as Mul<U16>>::Output: ArrayLength<(RunNode<Size>, RunKind)>
        + ArrayLength<CostNode<u16, RunNode<Size>>>
        + ArrayLength<Option<usize>>
        + ArrayLength<u16>
        + ArrayLength<Option<RunNode<Size>>>,
{
    use core::convert::TryInto;

    let maze = CheckedMaze::new(wall_manager, *config.cost_fn());
    RunCommander::new(
        state
            .current_node()
            .clone()
            .try_into()
            .expect("Should never panic"),
        config.goals(),
        maze,
    )
}

pub fn init_return_commander<'a, Size>(
    config: &Config<'a, Size>,
    state: &State<Size>,
    wall_manager: &'a WallManager<Size>,
) -> RunCommander<'a, Size>
where
    Size: Mul<Size> + Mul<U2> + Mul<U4> + Clone + PartialEq + PowerOfTwo + Unsigned,
    <Size as Mul<Size>>::Output: Mul<U2> + Mul<U16>,
    <Size as Mul<U4>>::Output: ArrayLength<(RunNode<Size>, u16)>
        + ArrayLength<WallNode<Wall<Size>, (RunNode<Size>, Pattern)>>,
    <<Size as Mul<Size>>::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
    <<Size as Mul<Size>>::Output as Mul<U16>>::Output: ArrayLength<(RunNode<Size>, RunKind)>
        + ArrayLength<CostNode<u16, RunNode<Size>>>
        + ArrayLength<Option<usize>>
        + ArrayLength<u16>
        + ArrayLength<Option<RunNode<Size>>>,
{
    use core::convert::TryInto;

    let maze = CheckedMaze::new(wall_manager, *config.cost_fn());
    RunCommander::new(
        state
            .current_node()
            .clone()
            .try_into()
            .expect("Should never panic"),
        &[config.return_goal().clone()],
        maze,
    )
}

pub fn init_search_agent<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    Size,
>(
    config: &'a Config<'a, Size>,
    state: &State<Size>,
    resource: Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor>,
    wall_manager: &'a WallManager<Size>,
) -> SearchAgent<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    Size,
>
where
    Math: crate::utils::math::Math,
    LeftEncoder: Encoder,
    RightEncoder: Encoder,
    Imu: IMU,
    LeftMotor: Motor,
    RightMotor: Motor,
    Size: Mul<Size>,
    Size::Output: Mul<U2>,
    <Size::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
{
    let robot = init_robot(config, state, resource, wall_manager);
    let command_converter = CommandConverter::new(*config.square_width(), *config.front_offset());
    let trajectory_manager =
        SearchTrajectoryManager::new(init_search_trajectory_generator(config), command_converter);
    SearchAgent::new(trajectory_manager, robot)
}

pub fn init_run_agent<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    Size,
>(
    config: &'a Config<'a, Size>,
    state: &State<Size>,
    resource: Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor>,
    wall_manager: &'a WallManager<Size>,
) -> RunAgent<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, Math, Size>
where
    Math: crate::utils::math::Math,
    LeftEncoder: Encoder,
    RightEncoder: Encoder,
    Imu: IMU,
    LeftMotor: Motor,
    RightMotor: Motor,
    Size: Mul<Size>,
    Size::Output: Mul<U2>,
    <Size::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
{
    let robot = init_robot(config, state, resource, wall_manager);
    let command_converter = CommandConverter::new(*config.square_width(), *config.front_offset());
    let trajectory_manager =
        TrackingTrajectoryManager::new(init_run_trajectory_generator(config), command_converter);
    RunAgent::new(trajectory_manager, robot)
}
