use crate::{
    agents::TrackingAgent,
    command_converter::CommandConverter,
    commanders::{
        ReturnCommander, ReturnSetupCommander, RunCommander, RunSetupCommander, SearchCommander,
    },
    controllers::MultiSisoController,
    estimator::Estimator,
    mazes::{CheckedMaze, Maze},
    nodes::{Node, Position, RunNode, SearchNode},
    operators::TrackingOperator,
    pattern_converters::DefaultPatternConverter,
    robot::Robot,
    solvers::dijkstra::DijkstraSolver,
    tracker::Tracker,
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
        Position<N>,
        Maze<WallManager<N>, DefaultPatternConverter<u16>, SearchNode<N>>,
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
            Tracker<MultiSisoController<LeftMotor, RightMotor>>,
            WallDetector<WallManager<N>, DistanceSensor, N>,
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
        Position<N>,
        CheckedMaze<WallManager<N>, DefaultPatternConverter<u16>, SearchNode<N>>,
        DijkstraSolver,
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
            Tracker<MultiSisoController<LeftMotor, RightMotor>>,
            WallDetector<WallManager<N>, DistanceSensor, N>,
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
        Position<N>,
        CheckedMaze<WallManager<N>, DefaultPatternConverter<u16>, SearchNode<N>>,
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
            Tracker<MultiSisoController<LeftMotor, RightMotor>>,
            WallDetector<WallManager<N>, DistanceSensor, N>,
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
        Position<N>,
        CheckedMaze<WallManager<N>, DefaultPatternConverter<u16>, SearchNode<N>>,
        DijkstraSolver,
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
            Tracker<MultiSisoController<LeftMotor, RightMotor>>,
            WallDetector<WallManager<N>, DistanceSensor, N>,
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
        Position<N>,
        CheckedMaze<WallManager<N>, DefaultPatternConverter<u16>, SearchNode<N>>,
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
            Tracker<MultiSisoController<LeftMotor, RightMotor>>,
            WallDetector<WallManager<N>, DistanceSensor, N>,
        >,
    >,
>;
