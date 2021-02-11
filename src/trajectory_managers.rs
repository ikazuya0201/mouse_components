mod search_trajectory_manager;
mod tracking_trajectory_manager;

pub use search_trajectory_manager::{
    SearchTrajectoryGenerator, TrajectoryManager as SearchTrajectoryManager,
};
pub use tracking_trajectory_manager::{
    InitialTrajectoryGenerator, TrajectoryManager as TrackingTrajectoryManager,
};

pub trait CommandConverter<Command> {
    type Output;

    fn convert(&self, source: &Command) -> Self::Output;
}
