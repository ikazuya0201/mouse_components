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
    const N: usize,
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
        wall_manager::WallManager<N>,
        obstacle_detector::ObstacleDetector<DistanceSensor, Math>,
        Math,
        N,
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
    const N: usize,
> = agents::TrackingAgent<
    trajectory_managers::TrackingTrajectoryManager<
        trajectory_generators::RunTrajectoryGenerator<
            Math,
            trajectory_generators::DefaultSlalomParametersGenerator,
        >,
        command_converter::CommandConverter,
        types::data::Target,
        trajectory_generators::ShiftTrajectory<trajectory_generators::RunTrajectory<Math>, Math>,
    >,
    Robot<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, Math, N>,
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
    const N: usize,
> = agents::TrackingAgent<
    trajectory_managers::TrackingTrajectoryManager<
        trajectory_generators::ReturnSetupTrajectoryGenerator,
        command_converter::ThroughCommandConverter,
        types::data::Target,
        trajectory_generators::SpinTrajectory,
    >,
    Robot<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, Math, N>,
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
        const N: usize,
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
        Robot<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, Math, N>,
    >;

pub type SearchCommander<'a, const N: usize> = commanders::SearchCommander<
    nodes::Node<N>,
    nodes::RunNode<N>,
    nodes::SearchNode<N>,
    types::data::SearchKind,
    mazes::Maze<
        'a,
        wall_manager::WallManager<N>,
        pattern_converters::LinearPatternConverter<u16>,
        nodes::SearchNode<N>,
    >,
>;

pub type RunCommander<'a, const N: usize> = commanders::RunCommander<
    nodes::RunNode<N>,
    mazes::CheckedMaze<
        'a,
        wall_manager::WallManager<N>,
        pattern_converters::LinearPatternConverter<u16>,
        nodes::SearchNode<N>,
    >,
>;

pub type ReturnSetupCommander<'a, const N: usize> = commanders::ReturnSetupCommander<
    nodes::RunNode<N>,
    mazes::CheckedMaze<
        'a,
        wall_manager::WallManager<N>,
        pattern_converters::LinearPatternConverter<u16>,
        nodes::SearchNode<N>,
    >,
>;
