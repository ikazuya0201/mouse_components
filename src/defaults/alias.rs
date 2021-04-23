use crate::{
    agents::TrackingAgent,
    command_converter::CommandConverter,
    commanders::{
        ReturnCommander, ReturnSetupCommander, RunCommander, RunSetupCommander, SearchCommander,
    },
    controllers::{RotationalController, TranslationalController},
    estimator::Estimator,
    mazes::Maze,
    nodes::{Node, RunNode, SearchNode},
    obstacle_detector::ObstacleDetector,
    operators::TrackingOperator,
    pattern_converters::LinearPatternConverter,
    robot::Robot,
    tracker::{RobotState, Tracker},
    trajectory_generators::{
        DefaultSlalomParametersGenerator, ReturnSetupTrajectoryGenerator, RunTrajectory,
        RunTrajectoryGenerator, SearchTrajectory, SearchTrajectoryGenerator, SetupTrajectory,
        ShiftTrajectory, Target,
    },
    trajectory_managers::{SearchTrajectoryManager, TrackingTrajectoryManager},
    types::data::SearchKind,
    wall_detector::WallDetector,
    wall_manager::WallManager,
};

pub type SearchOperator<
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    const N: usize,
> = TrackingOperator<
    SearchCommander<
        Node<N>,
        RunNode<N>,
        SearchNode<N>,
        SearchKind,
        Maze<WallManager<N>, LinearPatternConverter<u16>, SearchNode<N>>,
    >,
    TrackingAgent<
        SearchTrajectoryManager<
            SearchTrajectoryGenerator<Math>,
            CommandConverter,
            Target,
            ShiftTrajectory<SearchTrajectory<Math>, Math>,
        >,
        Robot<
            Estimator<LeftEncoder, RightEncoder, Imu, Math>,
            Tracker<LeftMotor, RightMotor, Math, TranslationalController, RotationalController>,
            WallDetector<WallManager<N>, ObstacleDetector<DistanceSensor, Math>, Math, N>,
            RobotState,
        >,
    >,
>;

pub type RunOperator<
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    const N: usize,
> = TrackingOperator<
    RunCommander<RunNode<N>, Maze<WallManager<N>, LinearPatternConverter<u16>, SearchNode<N>>>,
    TrackingAgent<
        TrackingTrajectoryManager<
            RunTrajectoryGenerator<Math, DefaultSlalomParametersGenerator>,
            CommandConverter,
            Target,
            ShiftTrajectory<RunTrajectory<Math>, Math>,
        >,
        Robot<
            Estimator<LeftEncoder, RightEncoder, Imu, Math>,
            Tracker<LeftMotor, RightMotor, Math, TranslationalController, RotationalController>,
            WallDetector<WallManager<N>, ObstacleDetector<DistanceSensor, Math>, Math, N>,
            RobotState,
        >,
    >,
>;

pub type ReturnSetupOperator<
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    const N: usize,
> = TrackingOperator<
    ReturnSetupCommander<
        RunNode<N>,
        Maze<WallManager<N>, LinearPatternConverter<u16>, SearchNode<N>>,
    >,
    TrackingAgent<
        TrackingTrajectoryManager<
            ReturnSetupTrajectoryGenerator<Math>,
            CommandConverter,
            Target,
            ShiftTrajectory<SetupTrajectory, Math>,
        >,
        Robot<
            Estimator<LeftEncoder, RightEncoder, Imu, Math>,
            Tracker<LeftMotor, RightMotor, Math, TranslationalController, RotationalController>,
            WallDetector<WallManager<N>, ObstacleDetector<DistanceSensor, Math>, Math, N>,
            RobotState,
        >,
    >,
>;

pub type ReturnOperator<
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    const N: usize,
> = TrackingOperator<
    ReturnCommander<RunNode<N>, Maze<WallManager<N>, LinearPatternConverter<u16>, SearchNode<N>>>,
    TrackingAgent<
        TrackingTrajectoryManager<
            RunTrajectoryGenerator<Math, DefaultSlalomParametersGenerator>,
            CommandConverter,
            Target,
            ShiftTrajectory<RunTrajectory<Math>, Math>,
        >,
        Robot<
            Estimator<LeftEncoder, RightEncoder, Imu, Math>,
            Tracker<LeftMotor, RightMotor, Math, TranslationalController, RotationalController>,
            WallDetector<WallManager<N>, ObstacleDetector<DistanceSensor, Math>, Math, N>,
            RobotState,
        >,
    >,
>;

pub type RunSetupOperator<
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    Math,
    const N: usize,
> = TrackingOperator<
    RunSetupCommander<RunNode<N>, Maze<WallManager<N>, LinearPatternConverter<u16>, SearchNode<N>>>,
    TrackingAgent<
        TrackingTrajectoryManager<
            ReturnSetupTrajectoryGenerator<Math>,
            CommandConverter,
            Target,
            ShiftTrajectory<SetupTrajectory, Math>,
        >,
        Robot<
            Estimator<LeftEncoder, RightEncoder, Imu, Math>,
            Tracker<LeftMotor, RightMotor, Math, TranslationalController, RotationalController>,
            WallDetector<WallManager<N>, ObstacleDetector<DistanceSensor, Math>, Math, N>,
            RobotState,
        >,
    >,
>;
