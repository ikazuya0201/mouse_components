#![cfg_attr(not(test), no_std)]
#![feature(iter_advance_by)]

mod administrator;
mod agents;
mod command_converter;
mod commander;
mod controller;
mod estimator;
mod maze;
mod node;
mod obstacle_detector;
pub mod operators;
mod pose_converter;
pub mod prelude;
mod tracker;
mod trajectory_generator;
pub mod utils;
mod wall_converter;
mod wall_manager;

pub mod traits {
    use super::*;

    pub use crate::utils::math::Math;
    pub use administrator::{Operator, OperatorStore, Selector};
    pub use agents::{
        ObstacleDetector, RunTrajectoryGenerator, SearchTrajectoryGenerator, StateEstimator,
        Tracker,
    };
    pub use commander::{
        BoundedNode, BoundedPathNode, Graph, GraphConverter, NextNode, NodeChecker,
        ObstacleInterpreter, RouteNode,
    };
    pub use maze::{
        GraphNode, PoseConverter, WallConverter, WallFinderNode, WallManager, WallSpaceNode,
    };
    pub use operators::{CommandConverter, RunAgent, RunCommander, SearchAgent, SearchCommander};
    pub use tracker::{Logger, RotationController, TranslationController};
}

pub mod sensors {
    use super::*;

    pub use estimator::{Encoder, IMU};
    pub use obstacle_detector::DistanceSensor;
    pub use tracker::Motor;
}

pub mod data_types {
    use super::*;

    pub use agents::Pose;
    pub use maze::{AbsoluteDirection, CorrectInfo, RelativeDirection};
    pub use node::Pattern;
    pub use obstacle_detector::Obstacle;
    pub use tracker::{AngleState, LengthState, State};
    pub use trajectory_generator::{
        AngleTarget, LengthTarget, MoveTarget, RunKind, SearchKind, SlalomDirection, SlalomKind,
        SlalomParameters, Target,
    };
    pub use wall_manager::Wall;
}

pub mod impls {
    use super::*;

    pub use administrator::Administrator;
    pub use agents::{RunAgent, SearchAgent};
    pub use command_converter::{CommandConverter, CommandConverter2};
    pub use commander::{RunCommander, SearchCommander};
    pub use controller::{
        RotationController, RotationControllerBuilder, TranslationController,
        TranslationControllerBuilder,
    };
    pub use estimator::{Estimator, EstimatorBuilder};
    pub use maze::Maze;
    pub use node::{Node, RunNode, SearchNode};
    pub use obstacle_detector::ObstacleDetector;
    pub use operators::{RunOperator, SearchOperator};
    pub use pose_converter::PoseConverter;
    pub use tracker::{NullLogger, Tracker, TrackerBuilder};
    pub use trajectory_generator::{
        slalom_parameters_map, slalom_parameters_map2, RunTrajectoryGenerator,
        RunTrajectoryGeneratorBuilder, SearchTrajectoryGenerator, SearchTrajectoryGeneratorBuilder,
        ShiftTrajectory,
    };
    pub use wall_converter::WallConverter;
    pub use wall_manager::WallManager;
}

pub mod errors {
    use super::*;

    pub use commander::{CannotCheckError, CommanderError, RunCommanderError};
    pub use operators::FinishError;
    pub use pose_converter::ConversionError;
}

pub mod defaults {
    use super::*;

    pub type SearchAgent<
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensor,
        Math,
        Logger = impls::NullLogger,
    > = impls::SearchAgent<
        impls::ObstacleDetector<DistanceSensor, Math>,
        impls::Estimator<LeftEncoder, RightEncoder, Imu, Math>,
        impls::Tracker<
            LeftMotor,
            RightMotor,
            Math,
            impls::TranslationController,
            impls::RotationController,
            Logger,
        >,
        impls::SearchTrajectoryGenerator<Math>,
        <impls::SearchTrajectoryGenerator<Math> as traits::SearchTrajectoryGenerator<(
            data_types::Pose,
            data_types::SearchKind,
        )>>::Trajectory,
        <impls::SearchTrajectoryGenerator<Math> as traits::SearchTrajectoryGenerator<(
            data_types::Pose,
            data_types::SearchKind,
        )>>::Target,
    >;

    pub type RunAgent<
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensor,
        Math,
        MaxPathLength,
        Logger = impls::NullLogger,
    > = impls::RunAgent<
        impls::ObstacleDetector<DistanceSensor, Math>,
        impls::Estimator<LeftEncoder, RightEncoder, Imu, Math>,
        impls::Tracker<
            LeftMotor,
            RightMotor,
            Math,
            impls::TranslationController,
            impls::RotationController,
            Logger,
        >,
        impls::RunTrajectoryGenerator<Math, MaxPathLength>,
        <impls::RunTrajectoryGenerator<Math, MaxPathLength> as traits::RunTrajectoryGenerator<(
            data_types::Pose,
            data_types::RunKind,
        )>>::Trajectory,
        MaxPathLength,
    >;

    pub type SearchCommander<Size, Math> = impls::SearchCommander<
        impls::Node<Size>,
        impls::RunNode<Size>,
        impls::SearchNode<Size>,
        data_types::SearchKind,
        impls::Maze<
            impls::WallManager<Size>,
            impls::PoseConverter<Size, Math>,
            impls::WallConverter,
            Math,
        >,
    >;

    pub type SearchOperator<
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensor,
        Math,
        Size,
        Logger = impls::NullLogger,
    > = impls::SearchOperator<
        data_types::Obstacle,
        (data_types::Pose, data_types::SearchKind),
        SearchAgent<
            LeftEncoder,
            RightEncoder,
            Imu,
            LeftMotor,
            RightMotor,
            DistanceSensor,
            Math,
            Logger,
        >,
        SearchCommander<Size, Math>,
        impls::CommandConverter2,
    >;

    pub type RunCommander<Size, Math> = impls::RunCommander<
        impls::RunNode<Size>,
        impls::Maze<
            impls::WallManager<Size>,
            impls::PoseConverter<Size, Math>,
            impls::WallConverter,
            Math,
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
        Size,
        Logger = impls::NullLogger,
    > = impls::RunOperator<
        RunAgent<
            LeftEncoder,
            RightEncoder,
            Imu,
            LeftMotor,
            RightMotor,
            DistanceSensor,
            Math,
            <impls::RunNode<Size> as traits::BoundedPathNode>::PathUpperBound,
            Logger,
        >,
        RunCommander<Size, Math>,
        impls::CommandConverter,
    >;
}
