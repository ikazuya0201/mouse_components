//! Implementations of [TrackingTrajectoryManager](crate::agents::TrackingTrajectoryManager).

mod search_trajectory_manager;
mod tracking_trajectory_manager;

pub use search_trajectory_manager::{
    SearchTrajectoryGenerator, TrajectoryManager as SearchTrajectoryManager,
    TrajectoryManagerError as SearchTrajectoryManagerError,
};
pub use tracking_trajectory_manager::{
    TrackingTrajectoryGenerator, TrajectoryManager as TrackingTrajectoryManager,
    TrajectoryManagerError as TrackingTrajectoryManagerError,
};

pub trait CommandConverter<Command> {
    type Output;

    fn convert(&self, source: &Command) -> Self::Output;
}
