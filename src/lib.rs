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

    #[doc(no_inline)]
    pub use crate::utils::math::Math;
    #[doc(no_inline)]
    pub use administrator::{Operator, OperatorStore, Selector};
    #[doc(no_inline)]
    pub use commanders::{Graph, NextNode, NodeChecker, RouteNode, UncheckedNodeFinder};
    #[doc(no_inline)]
    pub use mazes::{GraphNode, WallChecker, WallFinderNode, WallSpaceNode};
    #[doc(no_inline)]
    pub use operators::{TrackingAgent, TrackingCommander};
    #[doc(no_inline)]
    pub use tracker::Controller;
    #[doc(no_inline)]
    pub use wall_detector::ObstacleDetector;
}

/// Types in this crate.
pub mod types {
    use super::*;
    pub mod data {
        use super::*;

        #[doc(no_inline)]
        pub use nodes::{AbsoluteDirection, Pattern, RelativeDirection, RotationKind};
        #[doc(no_inline)]
        pub use obstacle_detector::Obstacle;
        #[doc(no_inline)]
        pub use tracker::{AngleState, LengthState, RobotState};
        #[doc(no_inline)]
        pub use trajectory_generators::Pose;
        #[doc(no_inline)]
        pub use trajectory_generators::{
            AngleTarget, LengthTarget, RunKind, SearchKind, SlalomDirection, SlalomKind,
            SlalomParameters, Target,
        };
        #[doc(no_inline)]
        pub use wall_manager::Wall;
    }
}

/// Traits of sensors.
pub mod sensors {
    use super::*;

    #[doc(no_inline)]
    pub use estimator::{Encoder, IMU};
    #[doc(no_inline)]
    pub use obstacle_detector::DistanceSensor;
    #[doc(no_inline)]
    pub use tracker::Motor;
}
