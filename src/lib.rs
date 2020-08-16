#![cfg_attr(not(test), no_std)]
#![feature(type_alias_impl_trait)]

mod administrator;
mod agent;
mod controller;
mod estimator;
mod maze;
mod obstacle_detector;
mod pattern;
pub mod prelude;
mod solver;
mod tracker;
mod trajectory_generator;
pub mod utils;

pub mod sensors {
    use super::*;

    pub use estimator::{Encoder, IMU};
    pub use tracker::Motor;
}

pub mod data_types {
    use super::*;

    pub use agent::Pose;
    pub use maze::{
        AbsoluteDirection, NodeId, RelativeDirection, SearchNodeId, WallDirection, WallPosition,
    };
    pub use obstacle_detector::Obstacle;
    pub use pattern::Pattern;
    pub use tracker::{State, SubState};
    pub use trajectory_generator::{SubTarget, Target};
}

pub mod impls {
    use super::*;

    pub use administrator::operator::SearchOperator;
    pub use administrator::Administrator;
    pub use agent::Agent;
    pub use controller::{Controller, ControllerBuilder};
    pub use estimator::{Estimator, EstimatorBuilder};
    pub use maze::{Maze, MazeBuilder};
    pub use obstacle_detector::ObstacleDetector;
    pub use solver::Solver;
    pub use tracker::{Tracker, TrackerBuilder};
    pub use trajectory_generator::{TrajectoryGenerator, TrajectoryGeneratorBuilder};
}
