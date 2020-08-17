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
    pub use obstacle_detector::distance_sensor::DistanceSensor;
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

pub mod defaults {
    use super::{data_types::*, impls::*};
    use quantities::{Angle, Distance};

    pub type DefaultTracker<LeftMotor, RightMotor> =
        Tracker<LeftMotor, RightMotor, Controller<Distance>, Controller<Angle>>;

    pub type DefaultAgent<
        LeftMotor,
        RightMotor,
        LeftEncoder,
        RightEncoder,
        Imu,
        DistanceSensor,
        DistanceSensorNum,
    > = Agent<
        State,
        Target,
        Pose,
        Obstacle,
        RelativeDirection,
        ObstacleDetector<DistanceSensor, DistanceSensorNum>,
        Estimator<LeftEncoder, RightEncoder, Imu>,
        DefaultTracker<LeftMotor, RightMotor>,
        TrajectoryGenerator,
    >;

    pub type DefaultMaze<MazeWidth> = Maze<MazeWidth, fn(Pattern) -> u16>;

    pub type DefaultSolver<MazeWidth, MaxSize, GoalSize> =
        Solver<NodeId<MazeWidth>, SearchNodeId<MazeWidth>, MaxSize, GoalSize>;

    pub type DefaultSearchOperator<
        'a,
        LeftMotor,
        RightMotor,
        LeftEncoder,
        RightEncoder,
        Imu,
        DistanceSensor,
        DistanceSensorNum,
        MazeWidth,
        MaxSize,
        GoalSize,
    > = SearchOperator<
        'a,
        NodeId<MazeWidth>,
        SearchNodeId<MazeWidth>,
        u16,
        RelativeDirection,
        Obstacle,
        Pose,
        DefaultMaze<MazeWidth>,
        DefaultAgent<
            LeftMotor,
            RightMotor,
            LeftEncoder,
            RightEncoder,
            Imu,
            DistanceSensor,
            DistanceSensorNum,
        >,
        DefaultSolver<MazeWidth, MaxSize, GoalSize>,
    >;
}
