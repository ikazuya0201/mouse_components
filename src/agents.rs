mod search_agent;
mod tracking_agent;

pub use search_agent::{SearchAgent, TrajectoryManager as SearchTrajectoryManager};
pub use tracking_agent::{TrackingAgent, TrajectoryManager as TrackingTrajectoryManager};

pub trait Robot<Target> {
    type Error;

    fn track_and_update(&mut self, target: &Target) -> Result<(), Self::Error>;
}
