mod run_trajectory_manager;
mod search_trajectory_manager;

pub use run_trajectory_manager::{
    RunTrajectoryGenerator, TrajectoryManager as RunTrajectoryManager,
};
pub use search_trajectory_manager::{
    SearchTrajectoryGenerator, TrajectoryManager as SearchTrajectoryManager,
};

pub trait CommandConverter<Command> {
    type Output;

    fn convert(&self, source: &Command) -> Self::Output;
}
