//! Components for implementing a code of [Micromouse contest](https://www.ntf.or.jp/mouse/).

#![cfg_attr(not(test), no_std)]
#![feature(iter_advance_by)]

pub mod administrator;
pub mod agents;
pub mod command_converter;
pub mod commanders;
pub mod config;
pub mod controllers;
pub mod estimator;
pub mod mazes;
pub mod nodes;
pub mod obstacle_detector;
pub mod operators;
pub mod pose_converter;
pub mod prelude;
pub mod robot;
pub mod state;
pub mod tracker;
pub mod trajectory_generators;
pub mod trajectory_managers;
pub mod utils;
pub mod wall_detector;
pub mod wall_manager;

pub mod traits {
    use super::*;

    pub use crate::utils::math::Math;
    pub use administrator::{Operator, OperatorStore, Selector};
    pub use commanders::{
        BoundedNode, BoundedPathNode, Graph, GraphConverter, NextNode, NodeChecker, RouteNode,
    };
    pub use mazes::{GraphNode, WallChecker, WallFinderNode, WallSpaceNode};
    pub use operators::{InitialCommander, TrackingAgent, TrackingInitializer};
    pub use tracker::{RotationController, TranslationController};
    pub use wall_detector::{CorrectInfo, ObstacleDetector, PoseConverter};
}

pub mod types {
    use super::*;
    pub mod data {
        use super::*;

        pub use nodes::{AbsoluteDirection, Pattern, RelativeDirection};
        pub use obstacle_detector::Obstacle;
        pub use tracker::{AngleState, LengthState, RobotState};
        pub use trajectory_generators::Pose;
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
}
