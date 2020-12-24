#![cfg_attr(not(test), no_std)]
#![feature(type_alias_impl_trait)]

extern crate alloc;

mod administrator;
mod agent;
mod controller;
mod estimator;
mod maze;
mod obstacle_detector;
mod operators;
pub mod prelude;
mod solver;
mod tracker;
mod trajectory_generator;
pub mod utils;

pub mod traits {
    use super::*;

    pub use administrator::{Atomic, Operator, OperatorStore, SelectMode, Selector};
    pub use agent::{ObstacleDetector, SearchTrajectoryGenerator, StateEstimator, Tracker};
    pub use operators::{
        run_operator::{RunAgent, RunCommander},
        search_operator::{SearchAgent, SearchCommander},
    };
    pub use solver::{Converter, ObstacleInterpreter};
    pub use tracker::{Logger, RotationController, TranslationController};
    pub use utils::math::Math;
}

pub mod sensors {
    use super::*;

    pub use estimator::{Encoder, IMU};
    pub use obstacle_detector::DistanceSensor;
    pub use tracker::Motor;
}

pub mod data_types {
    use super::*;

    pub use administrator::SelectMode;
    pub use agent::Pose;
    pub use maze::Pattern;
    pub use maze::{
        AbsoluteDirection, NodeId, Position, RelativeDirection, SearchNodeId, WallDirection,
        WallPosition,
    };
    pub use obstacle_detector::Obstacle;
    pub use tracker::{AngleState, LengthState, State};
    pub use trajectory_generator::{AngleTarget, LengthTarget, SearchKind, Target};
}

pub mod impls {
    use super::*;

    pub use administrator::Administrator;
    pub use agent::Agent;
    pub use controller::{
        RotationController, RotationControllerBuilder, TranslationController,
        TranslationControllerBuilder,
    };
    pub use estimator::{Estimator, EstimatorBuilder};
    pub use maze::{Maze, MazeBuilder};
    pub use obstacle_detector::ObstacleDetector;
    pub use operators::{run_operator::RunOperator, search_operator::SearchOperator};
    pub use solver::Solver;
    pub use tracker::{NullLogger, Tracker, TrackerBuilder};
    pub use trajectory_generator::{
        slalom_parameters_map, TrajectoryGenerator, TrajectoryGeneratorBuilder,
    };
}

pub mod defaults {
    use super::*;

    pub type Agent<
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensor,
        DistanceSensorNum,
        Math,
        Logger = impls::NullLogger,
    > = impls::Agent<
        impls::ObstacleDetector<DistanceSensor, DistanceSensorNum>,
        impls::Estimator<LeftEncoder, RightEncoder, Imu, Math>,
        impls::Tracker<
            LeftMotor,
            RightMotor,
            Math,
            impls::TranslationController,
            impls::RotationController,
            Logger,
        >,
        impls::TrajectoryGenerator<Math>,
    >;

    pub type Solver<MazeWidth, MaxPathLength, GoalSize, Math> = impls::Solver<
        data_types::NodeId<MazeWidth>,
        data_types::SearchNodeId<MazeWidth>,
        MaxPathLength,
        GoalSize,
        impls::Maze<MazeWidth, Math>,
    >;

    pub type SearchOperator<
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensor,
        DistanceSensorNum,
        MazeWidth,
        MaxPathLength,
        GoalSize,
        Mode,
        Math,
        Logger = impls::NullLogger,
    > = impls::SearchOperator<
        Mode,
        data_types::Obstacle,
        Agent<
            LeftEncoder,
            RightEncoder,
            Imu,
            LeftMotor,
            RightMotor,
            DistanceSensor,
            DistanceSensorNum,
            Math,
            Logger,
        >,
        Solver<MazeWidth, MaxPathLength, GoalSize, Math>,
    >;
}
