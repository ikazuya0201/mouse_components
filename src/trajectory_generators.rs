//! Implementations of trajectory generators.

mod return_setup_trajectory_generator;
mod run_trajectory_generator;
mod search_trajectory_generator;
mod slalom_generator;
mod spin_generator;
mod straight_generator;
mod trajectory;

pub use return_setup_trajectory_generator::{
    ReturnSetupTrajectoryGenerator, ReturnSetupTrajectoryGeneratorBuilder,
    ReturnSetupTrajectoryGeneratorConfig,
};
pub use run_trajectory_generator::{
    RunKind, RunTrajectory, RunTrajectoryGenerator, RunTrajectoryGeneratorBuilder,
    RunTrajectoryGeneratorConfig,
};
pub use search_trajectory_generator::{
    SearchKind, SearchTrajectory, SearchTrajectoryGenerator, SearchTrajectoryGeneratorBuilder,
    SearchTrajectoryGeneratorConfig,
};
pub use slalom_generator::{
    slalom_parameters_map, slalom_parameters_map2, SlalomDirection, SlalomKind, SlalomParameters,
    DEFAULT_ANGULAR_ACCELERATION_REF, DEFAULT_ANGULAR_JERK_REF, DEFAULT_ANGULAR_VELOCITY_REF,
};
pub use trajectory::{AngleTarget, LengthTarget, MoveTarget, ShiftTrajectory, Target};
use uom::si::f32::{Angle, Length};

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Pose {
    pub x: Length,
    pub y: Length,
    pub theta: Angle,
}

impl Pose {
    pub fn new(x: Length, y: Length, theta: Angle) -> Self {
        Self { x, y, theta }
    }
}
