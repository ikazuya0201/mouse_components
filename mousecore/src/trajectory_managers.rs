//! Implementations of [TrackingTrajectoryManager](crate::agents::TrackingTrajectoryManager).

mod search;
mod tracking;

pub use search::{
    SearchTrajectoryGenerator, TrajectoryManager as SearchTrajectoryManager,
    TrajectoryManagerError as SearchTrajectoryManagerError,
};
pub use tracking::{
    TrackingTrajectoryGenerator, TrajectoryManager as TrackingTrajectoryManager,
    TrajectoryManagerError as TrackingTrajectoryManagerError,
};

pub trait CommandConverter<Command> {
    type Output;

    fn convert(&self, source: &Command) -> Self::Output;
}
