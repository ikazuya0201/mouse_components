#![cfg_attr(not(test), no_std)]
#![feature(iter_advance_by)]

mod administrator;
pub mod agents;
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
pub mod trajectory_managers;
pub mod utils;
mod wall_converter;
pub mod wall_detector;
mod wall_manager;

pub mod traits {
    use super::*;

    pub use crate::utils::math::Math;
    pub use administrator::{Operator, OperatorStore, Selector};
    pub use commander::{
        BoundedNode, BoundedPathNode, Graph, GraphConverter, NextNode, NodeChecker, RouteNode,
    };
    pub use maze::{GraphNode, WallChecker, WallConverter, WallFinderNode, WallSpaceNode};
    pub use operators::{RunAgent, RunCommander};
    pub use tracker::{Logger, RotationController, TranslationController};
    pub use wall_detector::{CorrectInfo, ObstacleDetector, PoseConverter};
}

pub mod types {
    use super::*;
    pub mod data {
        use super::*;

        pub use agents::Pose;
        pub use maze::{AbsoluteDirection, RelativeDirection};
        pub use node::Pattern;
        pub use obstacle_detector::Obstacle;
        pub use tracker::{AngleState, LengthState, State};
        pub use trajectory_generator::{
            AngleTarget, LengthTarget, MoveTarget, RunKind, SearchKind, SlalomDirection,
            SlalomKind, SlalomParameters, Target,
        };
        pub use wall_manager::Wall;
    }
}

pub mod sensors {
    use super::*;

    pub use estimator::{Encoder, IMU};
    pub use obstacle_detector::DistanceSensor;
    pub use tracker::Motor;
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
        Logger = impls::NullLogger,
    > = impls::RunAgent<
        trajectory_managers::RunTrajectoryManager<
            (impls::RunNode<Size>, types::data::RunKind),
            impls::RunTrajectoryGenerator<Math, MaxPathLength>,
            impls::CommandConverter,
        >,
        robot::Robot<
            impls::Estimator<LeftEncoder, RightEncoder, Imu, Math>,
            impls::Tracker<
                LeftMotor,
                RightMotor,
                Math,
                impls::TranslationController,
                impls::RotationController,
                Logger,
            >,
            wall_detector::WallDetector<
                'a,
                impls::WallManager<Size>,
                impls::ObstacleDetector<DistanceSensor, Math>,
                impls::PoseConverter<Size, Math>,
                Math,
            >,
            types::data::State,
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
        Logger,
    > = agents::SearchAgent<
        trajectory_managers::SearchTrajectoryManager<
            impls::SearchTrajectoryGenerator<Math>,
            impls::CommandConverter,
            types::data::Target,
            <impls::SearchTrajectoryGenerator<Math> as trajectory_managers::SearchTrajectoryGenerator<(
                types::data::Pose,
                types::data::SearchKind,
            )>>::Trajectory,
        >,
        robot::Robot<
            impls::Estimator<LeftEncoder, RightEncoder, Imu, Math>,
            impls::Tracker<
                LeftMotor,
                RightMotor,
                Math,
                impls::TranslationController,
                impls::RotationController,
                Logger,
            >,
            wall_detector::WallDetector<
                'a,
                impls::WallManager<Size>,
                impls::ObstacleDetector<DistanceSensor, Math>,
                impls::PoseConverter<Size, Math>,
                Math,
            >,
            types::data::State,
        >,
    >;

    pub type SearchCommander<'a, Size> = impls::SearchCommander<
        impls::Node<Size>,
        impls::RunNode<Size>,
        impls::SearchNode<Size>,
        types::data::SearchKind,
        impls::Maze<'a, impls::WallManager<Size>, impls::WallConverter>,
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
        Logger = impls::NullLogger,
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
            Logger,
        >,
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
            'a,
            LeftEncoder,
            RightEncoder,
            Imu,
            LeftMotor,
            RightMotor,
            DistanceSensor,
            Size,
            Math,
            <impls::RunNode<Size> as traits::BoundedPathNode>::PathUpperBound,
            Logger,
        >,
        RunCommander<'a, Size>,
    >;
}
