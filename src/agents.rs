mod run_agent;
mod search_agent;

pub use run_agent::{RunAgent, TrajectoryManager as RunTrajectoryManager};
pub use search_agent::{SearchAgent, TrajectoryManager as SearchTrajectoryManager};

pub trait Robot<Target> {
    type Error;

    fn track_and_update(&mut self, target: &Target) -> Result<(), Self::Error>;
}
