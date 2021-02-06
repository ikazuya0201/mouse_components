mod run_agent;
mod search_agent;

use uom::si::f32::{Angle, Length};

pub use run_agent::{RunAgent, TrajectoryManager as RunTrajectoryManager};
pub use search_agent::{SearchAgent, TrajectoryManager as SearchTrajectoryManager};

pub trait Robot<Target> {
    type Error;

    fn track_and_update(&mut self, target: &Target) -> Result<(), Self::Error>;
}

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
