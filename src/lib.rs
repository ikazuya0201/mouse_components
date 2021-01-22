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
pub mod robot;
mod tracker;
mod trajectory_generator;
pub mod trajectory_manager;
pub mod utils;
mod wall_converter;
pub mod wall_detector;
mod wall_manager;

pub use agents::search_agent;

pub mod traits {
    use super::*;

    pub use crate::utils::math::Math;
    pub use administrator::{Operator, OperatorStore, Selector};
    pub use agents::{RunTrajectoryGenerator, StateEstimator, Tracker};
    pub use commander::{
        BoundedNode, BoundedPathNode, Graph, GraphConverter, NextNode, NodeChecker, RouteNode,
    };
    pub use maze::{GraphNode, WallConverter, WallFinderNode, WallManager, WallSpaceNode};
    pub use operators::{RunAgent, RunCommander};
    pub use tracker::{Logger, RotationController, TranslationController};
    pub use wall_detector::{CorrectInfo, ObstacleDetector, PoseConverter};
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
    pub use maze::{AbsoluteDirection, RelativeDirection};
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
    pub use agents::RunAgent;
    pub use command_converter::CommandConverter;
    pub use commander::{RunCommander, SearchCommander};
    pub use controller::{
        RotationController, RotationControllerBuilder, TranslationController,
        TranslationControllerBuilder,
    };
    pub use estimator::{Estimator, EstimatorBuilder};
    pub use maze::Maze;
    pub use node::{Node, RunNode, SearchNode};
    pub use obstacle_detector::ObstacleDetector;
    pub use operators::RunOperator;
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

    pub use commander::{CannotCheckError, RunCommanderError};
    pub use pose_converter::ConversionError;
}

pub mod defaults {
    use super::*;

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

    pub type SearchCommander<'a, Size> = impls::SearchCommander<
        impls::Node<Size>,
        impls::RunNode<Size>,
        impls::SearchNode<Size>,
        data_types::SearchKind,
        impls::Maze<'a, impls::WallManager<Size>, impls::WallConverter>,
    >;

    pub type RunCommander<'a, Size> = impls::RunCommander<
        impls::RunNode<Size>,
        impls::Maze<'a, impls::WallManager<Size>, impls::WallConverter>,
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
        RunCommander<'a, Size>,
        impls::CommandConverter,
    >;
}
