//! Components for implementing a code of [Micromouse contest](https://www.ntf.or.jp/mouse/).

#![cfg_attr(not(test), no_std)]
#![deny(warnings)]
#![feature(iter_advance_by)]

pub mod administrator;
pub mod agents;
pub mod command_converter;
pub mod commanders;
pub mod controllers;
pub mod defaults;
pub mod estimator;
pub mod mazes;
pub mod nodes;
pub mod obstacle_detector;
pub mod operators;
pub mod prelude;
pub mod robot;
pub mod tracker;
pub mod trajectory_generators;
pub mod trajectory_managers;
#[macro_use]
pub mod utils;
pub mod pattern_converters;
pub mod wall_detector;
pub mod wall_manager;

/// Traits in this crate.
pub mod traits {
    use super::*;

    pub use crate::utils::math::Math;
    pub use administrator::{Operator, OperatorStore, Selector};
    pub use commanders::{
        BoundedNode, Graph, NextNode, NodeChecker, RouteNode, UncheckedNodeFinder,
    };
    pub use mazes::{GraphNode, WallChecker, WallFinderNode, WallSpaceNode};
    pub use operators::{TrackingAgent, TrackingCommander};
    pub use tracker::Controller;
    pub use wall_detector::{CorrectInfo, ObstacleDetector, PoseConverter};
}

/// Types in this crate.
pub mod types {
    use super::*;
    pub mod data {
        use super::*;

        pub use nodes::{AbsoluteDirection, Pattern, RelativeDirection, RotationKind};
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

/// Traits of sensors.
pub mod sensors {
    use super::*;

    pub use estimator::{Encoder, IMU};
    pub use obstacle_detector::DistanceSensor;
    pub use tracker::Motor;
}
