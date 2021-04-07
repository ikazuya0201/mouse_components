//! Implementations of trajectory generators.

mod return_setup_trajectory_generator;
mod run_trajectory_generator;
mod search_trajectory_generator;
mod slalom_generator;
mod spin_generator;
mod straight_generator;
mod trajectory;

use serde::{Deserialize, Serialize};
use uom::si::f32::{Angle, Length};

pub use return_setup_trajectory_generator::{
    ReturnSetupTrajectoryGenerator, ReturnSetupTrajectoryGeneratorBuilder,
};
pub use run_trajectory_generator::{
    RunKind, RunTrajectory, RunTrajectoryGenerator, RunTrajectoryGeneratorBuilder,
    RunTrajectoryParameters,
};
pub use search_trajectory_generator::{
    SearchKind, SearchTrajectory, SearchTrajectoryGenerator, SearchTrajectoryGeneratorBuilder,
};
pub use slalom_generator::{
    DefaultSlalomParametersGenerator, SlalomDirection, SlalomKind, SlalomParameters,
    SlalomParametersGenerator,
};
pub use spin_generator::SpinTrajectory;
pub use trajectory::{AngleTarget, LengthTarget, ShiftTrajectory, Target};

#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize, Deserialize)]
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
