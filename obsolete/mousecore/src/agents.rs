//! Implementations of agents required by [operators](crate::operators).

mod tracking;

pub use tracking::{
    TrackingAgent, TrackingAgentError, TrajectoryManager as TrackingTrajectoryManager,
};

/// A trait that manages the actual machine.
pub trait Robot<Target> {
    type Error;

    fn track_and_update(&mut self, target: &Target) -> Result<(), Self::Error>;
    /// Stops all actuators in robot.
    fn stop(&mut self);
}
