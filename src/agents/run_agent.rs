use core::cell::RefCell;

use heapless::{spsc::Queue, ArrayLength};

use super::{ObstacleDetector, StateEstimator, Tracker};
use crate::maze::CorrectInfo;
use crate::operators::RunAgent as IRunAgent;

pub trait RunTrajectoryGenerator<Command> {
    type Target;
    type Trajectory: Iterator<Item = Self::Target>;
    type Trajectories: IntoIterator<Item = Self::Trajectory>;

    fn generate<Commands: IntoIterator<Item = Command>>(
        &self,
        commands: Commands,
    ) -> Self::Trajectories;
}

pub struct RunAgent<
    ObstacleDetectorType,
    StateEstimatorType,
    TrackerType,
    TrajectoryGeneratorType,
    Trajectory,
    MaxLength,
> where
    MaxLength: ArrayLength<Trajectory>,
{
    #[allow(unused)]
    obstacle_detector: ObstacleDetectorType,
    state_estimator: RefCell<StateEstimatorType>,
    tracker: RefCell<TrackerType>,
    trajectory_generator: TrajectoryGeneratorType,
    trajectories: RefCell<Queue<Trajectory, MaxLength>>,
}

impl<
        ObstacleDetectorType,
        StateEstimatorType,
        TrackerType,
        TrajectoryGeneratorType,
        Trajectory,
        MaxLength,
    >
    RunAgent<
        ObstacleDetectorType,
        StateEstimatorType,
        TrackerType,
        TrajectoryGeneratorType,
        Trajectory,
        MaxLength,
    >
where
    MaxLength: ArrayLength<Trajectory>,
{
    pub fn new(
        obstacle_detector: ObstacleDetectorType,
        state_estimator: StateEstimatorType,
        tracker: TrackerType,
        trajectory_generator: TrajectoryGeneratorType,
    ) -> Self {
        Self {
            obstacle_detector,
            state_estimator: RefCell::new(state_estimator),
            tracker: RefCell::new(tracker),
            trajectory_generator,
            trajectories: RefCell::new(Queue::new()),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RunAgentError<T> {
    TrackFinish,
    TrackFailed(T),
}

impl<
        Command,
        ObstacleDetectorType,
        StateEstimatorType,
        TrackerType,
        TrajectoryGeneratorType,
        MaxLength,
    > IRunAgent<Command>
    for RunAgent<
        ObstacleDetectorType,
        StateEstimatorType,
        TrackerType,
        TrajectoryGeneratorType,
        TrajectoryGeneratorType::Trajectory,
        MaxLength,
    >
where
    ObstacleDetectorType: ObstacleDetector<StateEstimatorType::State>,
    StateEstimatorType: StateEstimator<CorrectInfo>,
    TrackerType: Tracker<StateEstimatorType::State, TrajectoryGeneratorType::Target>,
    TrajectoryGeneratorType: RunTrajectoryGenerator<Command>,
    MaxLength: ArrayLength<TrajectoryGeneratorType::Trajectory>,
{
    type Error = RunAgentError<TrackerType::Error>;

    fn set_commands<Commands: IntoIterator<Item = Command>>(&self, commands: Commands) {
        for trajectory in self.trajectory_generator.generate(commands) {
            self.trajectories
                .borrow_mut()
                .enqueue(trajectory)
                .unwrap_or_else(|_| {
                    unreachable!("The length of trajectory queue should never be exceeded.")
                });
        }
    }

    fn track_next(&self) -> Result<(), Self::Error> {
        let mut trajectories = self.trajectories.borrow_mut();
        loop {
            if trajectories.is_empty() {
                self.tracker.borrow_mut().stop();
                return Err(RunAgentError::TrackFinish);
            }
            let trajectory = trajectories.iter_mut().next().unwrap();
            if let Some(target) = trajectory.next() {
                self.state_estimator.borrow_mut().estimate();
                let state = self.state_estimator.borrow().state();
                if let Err(err) = self.tracker.borrow_mut().track(&state, &target) {
                    self.tracker.borrow_mut().stop();
                    return Err(RunAgentError::TrackFailed(err));
                }
                return Ok(());
            }
            trajectories.dequeue();
        }
    }
}
