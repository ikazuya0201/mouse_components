//! Implementations of agents required by [operators](crate::operators).

mod tracking_agent;

pub use tracking_agent::{
    TrackingAgent, TrackingAgentError, TrajectoryManager as TrackingTrajectoryManager,
};

/// A trait that manages the actual machine.
pub trait Robot<Target> {
    type Error;

    fn track_and_update(&mut self, target: &Target) -> Result<(), Self::Error>;
}
