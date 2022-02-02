//! Implementations of trajectory generators.

mod run;
mod search;
mod setup;
mod slalom;
mod spin;
mod straight;
mod trajectory;

use serde::{Deserialize, Serialize};
use uom::si::f32::{Angle, Length};

pub use self::spin::SpinTrajectory;
pub use run::{
    RunKind, RunState, RunTrajectory, RunTrajectoryGenerator, RunTrajectoryGeneratorBuilder,
    RunTrajectoryGeneratorConfig, RunTrajectoryParameters,
};
pub use search::{
    SearchKind, SearchTrajectory, SearchTrajectoryGenerator, SearchTrajectoryGeneratorBuilder,
    SearchTrajectoryGeneratorConfig,
};
pub use setup::{
    ReturnSetupTrajectoryGenerator, ReturnSetupTrajectoryGeneratorBuilder,
    ReturnSetupTrajectoryGeneratorConfig, SetupTrajectory,
};
pub use slalom::{
    DefaultSlalomParametersGenerator, SlalomDirection, SlalomKind, SlalomParameters,
    SlalomParametersGenerator,
};
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
