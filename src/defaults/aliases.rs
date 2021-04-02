use crate::{
    agents, command_converter, commanders, controllers, estimator, mazes, nodes, obstacle_detector,
    pattern_converters, robot, tracker, trajectory_generators, trajectory_managers, types,
    wall_detector, wall_manager,
};

pub type Robot<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    Size,
> = robot::Robot<
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

pub type RunAgent<
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
        trajectory_generators::RunTrajectoryGenerator<
            Math,
            trajectory_generators::SlalomParametersGeneratorWithFrontOffset,
        >,
        command_converter::CommandConverter,
        types::data::Target,
        trajectory_generators::ShiftTrajectory<trajectory_generators::RunTrajectory<Math>, Math>,
    >,
    Robot<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, Math, Size>,
>;

pub type ReturnSetupAgent<
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
        trajectory_generators::ReturnSetupTrajectoryGenerator,
        command_converter::ThroughCommandConverter,
        types::data::Target,
        trajectory_generators::SpinTrajectory,
    >,
    Robot<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, Math, Size>,
>;

pub type SearchAgent<
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

pub type SearchCommander<'a, Size> = commanders::SearchCommander<
    nodes::Node<Size>,
    nodes::RunNode<Size>,
    nodes::SearchNode<Size>,
    types::data::SearchKind,
    mazes::Maze<
        'a,
        wall_manager::WallManager<Size>,
        pattern_converters::LinearPatternConverter<u16>,
        nodes::SearchNode<Size>,
    >,
>;

pub type RunCommander<'a, Size> = commanders::RunCommander<
    nodes::RunNode<Size>,
    mazes::CheckedMaze<
        'a,
        wall_manager::WallManager<Size>,
        pattern_converters::LinearPatternConverter<u16>,
        nodes::SearchNode<Size>,
    >,
>;

pub type ReturnSetupCommander<'a, Size> = commanders::ReturnSetupCommander<
    nodes::RunNode<Size>,
    mazes::CheckedMaze<
        'a,
        wall_manager::WallManager<Size>,
        pattern_converters::LinearPatternConverter<u16>,
        nodes::SearchNode<Size>,
    >,
>;
