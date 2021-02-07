mod run_trajectory_generator;
mod search_trajectory_generator;
mod slalom_generator;
mod spin_generator;
mod straight_generator;
mod trajectory;

pub use run_trajectory_generator::{
    RunKind, RunTrajectory, RunTrajectoryGenerator, RunTrajectoryGeneratorBuilder,
};
pub use search_trajectory_generator::{
    SearchKind, SearchTrajectory, SearchTrajectoryGenerator, SearchTrajectoryGeneratorBuilder,
};
pub use slalom_generator::{
    slalom_parameters_map, slalom_parameters_map2, SlalomDirection, SlalomKind, SlalomParameters,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RequiredFieldEmptyError {
    field_name: &'static str,
}

impl core::fmt::Display for RequiredFieldEmptyError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(
            f,
            "Build failed: the field `{}` is required",
            self.field_name
        )
    }
}

fn ok_or<T>(value: Option<T>, field_name: &'static str) -> Result<T, RequiredFieldEmptyError> {
    value.ok_or(RequiredFieldEmptyError { field_name })
}
