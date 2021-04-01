pub mod initialize;
mod return_operator;
mod return_setup_operator;
mod run_operator;
mod search_operator;

use crate::{
    agents, command_converter, commanders, controllers, estimator, mazes, nodes, obstacle_detector,
    robot, tracker, trajectory_generators, trajectory_managers, types, wall_detector, wall_manager,
};
pub use return_operator::ReturnOperator;
pub use return_setup_operator::ReturnSetupOperator;
pub use run_operator::RunOperator;
pub use search_operator::SearchOperator;

type Robot<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, Math, Size> =
    robot::Robot<
        estimator::Estimator<LeftEncoder, RightEncoder, Imu, Math>,
        tracker::Tracker<
            LeftMotor,
            RightMotor,
            Math,
            controllers::TranslationalController,
            controllers::RotationalController,
        >,
        wall_detector::WallDetector<
            'a,
            wall_manager::WallManager<Size>,
            obstacle_detector::ObstacleDetector<DistanceSensor, Math>,
            Math,
            Size,
        >,
        types::data::RobotState,
    >;

type RunAgent<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    Size,
> = agents::TrackingAgent<
    trajectory_managers::TrackingTrajectoryManager<
        (nodes::RunNode<Size>, types::data::RunKind),
        trajectory_generators::RunTrajectoryGenerator<Math>,
        types::data::Target,
        command_converter::CommandConverter,
    >,
    Robot<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, Math, Size>,
>;

type ReturnSetupAgent<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    Size,
> = agents::TrackingAgent<
    trajectory_managers::TrackingTrajectoryManager<
        types::data::RotationKind,
        trajectory_generators::ReturnSetupTrajectoryGenerator<Math>,
        types::data::Target,
        command_converter::ThroughCommandConverter,
    >,
    Robot<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, Math, Size>,
>;

type SearchAgent<
        'a,
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensor,
        Math,
        Size,
    > = agents::TrackingAgent<
        trajectory_managers::SearchTrajectoryManager<
            trajectory_generators::SearchTrajectoryGenerator<Math>,
            command_converter::CommandConverter,
            types::data::Target,
            <trajectory_generators::SearchTrajectoryGenerator<Math> as trajectory_managers::SearchTrajectoryGenerator<(
                types::data::Pose,
                types::data::SearchKind,
            )>>::Trajectory,
        >,
        Robot<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, Math, Size>,
    >;

type SearchCommander<'a, Size> = commanders::SearchCommander<
    nodes::Node<Size>,
    nodes::RunNode<Size>,
    nodes::SearchNode<Size>,
    types::data::SearchKind,
    mazes::Maze<
        'a,
        wall_manager::WallManager<Size>,
        types::data::Pattern,
        u16,
        nodes::SearchNode<Size>,
    >,
>;

type RunCommander<'a, Size> = commanders::RunCommander<
    nodes::RunNode<Size>,
    mazes::CheckedMaze<
        'a,
        wall_manager::WallManager<Size>,
        types::data::Pattern,
        u16,
        nodes::SearchNode<Size>,
    >,
>;

type ReturnSetupCommander<'a, Size> = commanders::ReturnSetupCommander<
    nodes::RunNode<Size>,
    mazes::CheckedMaze<
        'a,
        wall_manager::WallManager<Size>,
        types::data::Pattern,
        u16,
        nodes::SearchNode<Size>,
    >,
>;
