#![cfg_attr(not(test), no_std)]
#![feature(iter_advance_by)]

pub mod administrator;
pub mod agents;
pub mod command_converter;
pub mod commanders;
pub mod controllers;
pub mod estimator;
pub mod maze;
pub mod node;
pub mod obstacle_detector;
pub mod operators;
pub mod pose_converter;
pub mod prelude;
pub mod robot;
pub mod tracker;
pub mod trajectory_generators;
pub mod trajectory_managers;
pub mod utils;
pub mod wall_converter;
pub mod wall_detector;
pub mod wall_manager;

pub mod traits {
    use super::*;

    pub use crate::utils::math::Math;
    pub use administrator::{Operator, OperatorStore, Selector};
    pub use commanders::{
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
        pub use trajectory_generators::{
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
        Logger = tracker::NullLogger,
    > = agents::RunAgent<
        trajectory_managers::RunTrajectoryManager<
            (node::RunNode<Size>, types::data::RunKind),
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
                Logger,
            >,
            wall_detector::WallDetector<
                'a,
                wall_manager::WallManager<Size>,
                obstacle_detector::ObstacleDetector<DistanceSensor, Math>,
                pose_converter::PoseConverter<Size, Math>,
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
                Logger,
            >,
            wall_detector::WallDetector<
                'a,
                wall_manager::WallManager<Size>,
                obstacle_detector::ObstacleDetector<DistanceSensor, Math>,
                pose_converter::PoseConverter<Size, Math>,
                Math,
            >,
            types::data::State,
        >,
    >;

    pub type SearchCommander<'a, Size> = commanders::SearchCommander<
        node::Node<Size>,
        node::RunNode<Size>,
        node::SearchNode<Size>,
        types::data::SearchKind,
        maze::Maze<'a, wall_manager::WallManager<Size>, wall_converter::WallConverter>,
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
        Logger = tracker::NullLogger,
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

    pub type RunCommander<'a, Size> = commanders::RunCommander<
        node::RunNode<Size>,
        maze::Maze<'a, wall_manager::WallManager<Size>, wall_converter::WallConverter>,
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
        Logger = tracker::NullLogger,
    > = operators::RunOperator<
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
            <node::RunNode<Size> as traits::BoundedPathNode>::PathUpperBound,
            Logger,
        >,
        RunCommander<'a, Size>,
    >;
}
