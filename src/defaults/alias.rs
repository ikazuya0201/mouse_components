use crate::{
    agents::TrackingAgent,
    command_converter::CommandConverter,
    commanders::{
        ReturnCommander, ReturnSetupCommander, RunCommander, RunSetupCommander, SearchCommander,
    },
    controllers::{RotationalController, TranslationalController},
    estimator::Estimator,
    mazes::{CheckedMaze, Maze},
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
            SearchTrajectoryGenerator,
            CommandConverter,
            Target,
            ShiftTrajectory<SearchTrajectory>,
        >,
        Robot<
            Estimator<LeftEncoder, RightEncoder, Imu>,
            Tracker<LeftMotor, RightMotor, TranslationalController, RotationalController>,
            WallDetector<WallManager<N>, ObstacleDetector<DistanceSensor>, N>,
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
    const N: usize,
> = TrackingOperator<
    RunCommander<
        RunNode<N>,
        CheckedMaze<WallManager<N>, LinearPatternConverter<u16>, SearchNode<N>>,
    >,
    TrackingAgent<
        TrackingTrajectoryManager<
            RunTrajectoryGenerator<DefaultSlalomParametersGenerator>,
            CommandConverter,
            Target,
            ShiftTrajectory<RunTrajectory>,
        >,
        Robot<
            Estimator<LeftEncoder, RightEncoder, Imu>,
            Tracker<LeftMotor, RightMotor, TranslationalController, RotationalController>,
            WallDetector<WallManager<N>, ObstacleDetector<DistanceSensor>, N>,
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
    const N: usize,
> = TrackingOperator<
    ReturnSetupCommander<
        RunNode<N>,
        CheckedMaze<WallManager<N>, LinearPatternConverter<u16>, SearchNode<N>>,
    >,
    TrackingAgent<
        TrackingTrajectoryManager<
            ReturnSetupTrajectoryGenerator,
            CommandConverter,
            Target,
            ShiftTrajectory<SetupTrajectory>,
        >,
        Robot<
            Estimator<LeftEncoder, RightEncoder, Imu>,
            Tracker<LeftMotor, RightMotor, TranslationalController, RotationalController>,
            WallDetector<WallManager<N>, ObstacleDetector<DistanceSensor>, N>,
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
    const N: usize,
> = TrackingOperator<
    ReturnCommander<
        RunNode<N>,
        CheckedMaze<WallManager<N>, LinearPatternConverter<u16>, SearchNode<N>>,
    >,
    TrackingAgent<
        TrackingTrajectoryManager<
            RunTrajectoryGenerator<DefaultSlalomParametersGenerator>,
            CommandConverter,
            Target,
            ShiftTrajectory<RunTrajectory>,
        >,
        Robot<
            Estimator<LeftEncoder, RightEncoder, Imu>,
            Tracker<LeftMotor, RightMotor, TranslationalController, RotationalController>,
            WallDetector<WallManager<N>, ObstacleDetector<DistanceSensor>, N>,
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
    const N: usize,
> = TrackingOperator<
    RunSetupCommander<
        RunNode<N>,
        CheckedMaze<WallManager<N>, LinearPatternConverter<u16>, SearchNode<N>>,
    >,
    TrackingAgent<
        TrackingTrajectoryManager<
            ReturnSetupTrajectoryGenerator,
            CommandConverter,
            Target,
            ShiftTrajectory<SetupTrajectory>,
        >,
        Robot<
            Estimator<LeftEncoder, RightEncoder, Imu>,
            Tracker<LeftMotor, RightMotor, TranslationalController, RotationalController>,
            WallDetector<WallManager<N>, ObstacleDetector<DistanceSensor>, N>,
            RobotState,
        >,
    >,
>;
