mod run_agent;
mod search_agent;

use uom::si::f32::{Angle, Length};

pub use run_agent::{RunAgent, RunAgentError, RunTrajectoryGenerator};
pub use search_agent::{SearchAgent, SearchAgentError, SearchTrajectoryGenerator};

pub trait ObstacleDetector<State> {
    type Obstacle;
    type Obstacles: IntoIterator<Item = Self::Obstacle>;

    fn detect(&mut self, state: &State) -> Self::Obstacles;
}

pub trait StateEstimator<Diff> {
    type State;

    fn init(&mut self);
    fn estimate(&mut self);
    fn state(&self) -> Self::State;
    fn correct_state<Diffs: IntoIterator<Item = Diff>>(&mut self, diffs: Diffs);
}

pub trait Tracker<State, Target> {
    type Error;

    fn init(&mut self);
    fn track(&mut self, state: &State, target: &Target) -> Result<(), Self::Error>;
    fn stop(&mut self);
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
