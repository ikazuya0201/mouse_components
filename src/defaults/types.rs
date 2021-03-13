use crate::{
    agents, command_converter, commanders, controllers, estimator, mazes, nodes, obstacle_detector,
    operators, pose_converter, robot, tracker, traits, trajectory_generators, trajectory_managers,
    types, wall_detector, wall_manager,
};

pub type RunAgent<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Size,
    Math,
    MaxPathLength,
> = agents::TrackingAgent<
    trajectory_managers::TrackingTrajectoryManager<
        (nodes::RunNode<Size>, types::data::RunKind),
        trajectory_generators::RunTrajectoryGenerator<Math, MaxPathLength>,
        command_converter::CommandConverter,
    >,
    robot::Robot<
        estimator::Estimator<LeftEncoder, RightEncoder, Imu, Math>,
        tracker::Tracker<
            LeftMotor,
            RightMotor,
            Math,
            controllers::TranslationController,
            controllers::RotationController,
        >,
        wall_detector::WallDetector<
            'a,
            wall_manager::WallManager<Size>,
            obstacle_detector::ObstacleDetector<DistanceSensor, Math>,
            pose_converter::PoseConverter<Size, Math>,
            Math,
        >,
        types::data::RobotState,
    >,
>;

pub type ReturnSetupAgent<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Size,
    Math,
> = agents::TrackingAgent<
    trajectory_managers::TrackingTrajectoryManager<
        types::data::RotationKind,
        trajectory_generators::ReturnSetupTrajectoryGenerator<Math>,
        command_converter::ThroughCommandConverter,
    >,
    robot::Robot<
        estimator::Estimator<LeftEncoder, RightEncoder, Imu, Math>,
        tracker::Tracker<
            LeftMotor,
            RightMotor,
            Math,
            controllers::TranslationController,
            controllers::RotationController,
        >,
        wall_detector::WallDetector<
            'a,
            wall_manager::WallManager<Size>,
            obstacle_detector::ObstacleDetector<DistanceSensor, Math>,
            pose_converter::PoseConverter<Size, Math>,
            Math,
        >,
        types::data::RobotState,
    >,
>;

pub type SearchAgent<
        'a,
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensor,
        Size,
        Math,
    > = agents::SearchAgent<
        trajectory_managers::SearchTrajectoryManager<
            trajectory_generators::SearchTrajectoryGenerator<Math>,
            command_converter::CommandConverter,
            types::data::Target,
            <trajectory_generators::SearchTrajectoryGenerator<Math> as trajectory_managers::SearchTrajectoryGenerator<(
                types::data::Pose,
                types::data::SearchKind,
            )>>::Trajectory,
        >,
        robot::Robot<
            estimator::Estimator<LeftEncoder, RightEncoder, Imu, Math>,
            tracker::Tracker<
                LeftMotor,
                RightMotor,
                Math,
                controllers::TranslationController,
                controllers::RotationController,
            >,
            wall_detector::WallDetector<
                'a,
                wall_manager::WallManager<Size>,
                obstacle_detector::ObstacleDetector<DistanceSensor, Math>,
                pose_converter::PoseConverter<Size, Math>,
                Math,
            >,
            types::data::RobotState,
        >,
    >;

pub type SearchCommander<'a, Size> = commanders::SearchCommander<
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

pub type SearchOperator<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    Size,
> = operators::SearchOperator<
    SearchCommander<'a, Size>,
    SearchAgent<
        'a,
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensor,
        Size,
        Math,
    >,
>;

pub type RunCommander<'a, Size> = commanders::RunCommander<
    nodes::RunNode<Size>,
    mazes::CheckedMaze<
        'a,
        wall_manager::WallManager<Size>,
        types::data::Pattern,
        u16,
        nodes::SearchNode<Size>,
    >,
>;

pub type ReturnCommander<'a, Size> = commanders::ReturnCommander<
    nodes::RunNode<Size>,
    mazes::CheckedMaze<
        'a,
        wall_manager::WallManager<Size>,
        types::data::Pattern,
        u16,
        nodes::SearchNode<Size>,
    >,
>;

pub type ReturnSetupCommander<'a, Size> = commanders::ReturnSetupCommander<
    nodes::RunNode<Size>,
    mazes::CheckedMaze<
        'a,
        wall_manager::WallManager<Size>,
        types::data::Pattern,
        u16,
        nodes::SearchNode<Size>,
    >,
>;

pub type RunOperator<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    Size,
> = operators::TrackingOperator<
    RunCommander<'a, Size>,
    RunAgent<
        'a,
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensor,
        Size,
        Math,
        <nodes::RunNode<Size> as traits::BoundedPathNode>::PathUpperBound,
    >,
>;

pub type ReturnOperator<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    Size,
> = operators::TrackingOperator<
    ReturnCommander<'a, Size>,
    RunAgent<
        'a,
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensor,
        Size,
        Math,
        <nodes::RunNode<Size> as traits::BoundedPathNode>::PathUpperBound,
    >,
>;

pub type ReturnSetupOperator<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    Size,
> = operators::TrackingOperator<
    ReturnSetupCommander<'a, Size>,
    ReturnSetupAgent<
        'a,
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensor,
        Size,
        Math,
    >,
>;
