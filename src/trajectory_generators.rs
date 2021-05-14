//! Implementations of trajectory generators.

mod run_trajectory_generator;
mod search_trajectory_generator;
mod setup_trajectory_generator;
mod slalom_generator;
mod spin_generator;
mod straight_generator;
mod trajectory;

use serde::{Deserialize, Serialize};
use uom::si::f32::{Angle, Length};

pub use run_trajectory_generator::{
    RunKind, RunState, RunTrajectory, RunTrajectoryGenerator, RunTrajectoryGeneratorBuilder,
    RunTrajectoryGeneratorConfig, RunTrajectoryParameters,
};
pub use search_trajectory_generator::{
    SearchKind, SearchTrajectory, SearchTrajectoryGenerator, SearchTrajectoryGeneratorBuilder,
    SearchTrajectoryGeneratorConfig,
};
pub use setup_trajectory_generator::{
    ReturnSetupTrajectoryGenerator, ReturnSetupTrajectoryGeneratorBuilder,
    ReturnSetupTrajectoryGeneratorConfig, SetupTrajectory,
};
pub use slalom_generator::{
    DefaultSlalomParametersGenerator, SlalomDirection, SlalomKind, SlalomParameters,
    SlalomParametersGenerator,
};
pub use spin_generator::SpinTrajectory;
pub use trajectory::{AngleTarget, LengthTarget, ShiftTrajectory, SingleTrajectory, Target};

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
