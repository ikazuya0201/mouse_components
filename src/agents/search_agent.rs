use core::cell::{Cell, RefCell};

use heapless::spsc::Queue;
use spin::Mutex;
use typenum::consts::*;

use super::{ObstacleDetector, StateEstimator, Tracker};
use crate::operators::{EmptyTrajectoyError, SearchAgent as ISearchAgent};

pub trait SearchTrajectoryGenerator<Command> {
    type Target;
    type Trajectory: Iterator<Item = Self::Target>;

    fn generate_search(&self, command: &Command) -> Self::Trajectory;
    fn generate_emergency(&self, target: &Self::Target) -> Self::Trajectory;
}

//TODO: separate Agent to SearchAgent and RunAgent with AgentInner
//Initialize RunAgent from SearchAgent by implementing From<SearchAgent>
pub struct SearchAgent<
    IObstacleDetector,
    IStateEstimator,
    ITracker,
    ITrajectoryGenerator,
    Trajectory,
    Target,
> {
    obstacle_detector: RefCell<IObstacleDetector>,
    state_estimator: RefCell<IStateEstimator>,
    tracker: Mutex<ITracker>,
    trajectory_generator: ITrajectoryGenerator,
    trajectories: Mutex<Queue<Trajectory, U2>>,
    emergency_trajectory: RefCell<Option<Trajectory>>,
    emergency_counter: Cell<usize>,
    last_target: RefCell<Option<Target>>,
}

impl<IObstacleDetector, IStateEstimator, ITracker, ITrajectoryGenerator, Trajectory, Target>
    core::fmt::Debug
    for SearchAgent<
        IObstacleDetector,
        IStateEstimator,
        ITracker,
        ITrajectoryGenerator,
        Trajectory,
        Target,
    >
where
    ITracker: core::fmt::Debug,
    IStateEstimator: core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(
            f,
            "Agent{{ tracker:{:?}, estimator: {:?} }}",
            self.tracker, self.state_estimator,
        )
    }
}

impl<IObstacleDetector, IStateEstimator, ITracker, ITrajectoryGenerator, Trajectory, Target>
    SearchAgent<
        IObstacleDetector,
        IStateEstimator,
        ITracker,
        ITrajectoryGenerator,
        Trajectory,
        Target,
    >
{
    pub fn new(
        obstacle_detector: IObstacleDetector,
        state_estimator: IStateEstimator,
        tracker: ITracker,
        trajectory_generator: ITrajectoryGenerator,
    ) -> Self {
        Self {
            obstacle_detector: RefCell::new(obstacle_detector),
            state_estimator: RefCell::new(state_estimator),
            tracker: Mutex::new(tracker),
            trajectory_generator,
            trajectories: Mutex::new(Queue::new()),
            emergency_trajectory: RefCell::new(None),
            emergency_counter: Cell::new(0),
            last_target: RefCell::new(None),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum SearchAgentError<T> {
    TrackFailed(T),
    QueueOverflowed,
    EmptyTrajectory,
}

impl<T> core::convert::TryFrom<SearchAgentError<T>> for EmptyTrajectoyError {
    type Error = SearchAgentError<T>;

    fn try_from(value: SearchAgentError<T>) -> Result<Self, SearchAgentError<T>> {
        match value {
            SearchAgentError::EmptyTrajectory => Ok(EmptyTrajectoyError),
            _ => Err(value),
        }
    }
}

impl<T> From<T> for SearchAgentError<T> {
    fn from(value: T) -> Self {
        SearchAgentError::TrackFailed(value)
    }
}

impl<Command, Diff, IObstacleDetector, IStateEstimator, ITracker, ITrajectoryGenerator>
    ISearchAgent<Command, Diff>
    for SearchAgent<
        IObstacleDetector,
        IStateEstimator,
        ITracker,
        ITrajectoryGenerator,
        ITrajectoryGenerator::Trajectory,
        ITrajectoryGenerator::Target,
    >
where
    IObstacleDetector: ObstacleDetector<IStateEstimator::State>,
    IStateEstimator: StateEstimator<Diff>,
    ITracker: Tracker<IStateEstimator::State, ITrajectoryGenerator::Target>,
    ITrajectoryGenerator: SearchTrajectoryGenerator<Command>,
{
    type Error = SearchAgentError<ITracker::Error>;
    type Obstacle = IObstacleDetector::Obstacle;
    type Obstacles = IObstacleDetector::Obstacles;

    fn update_state(&self) {
        self.state_estimator.borrow_mut().estimate();
    }

    fn get_obstacles(&self) -> Self::Obstacles {
        self.obstacle_detector
            .borrow_mut()
            .detect(self.state_estimator.borrow().state())
    }

    fn set_command(&self, command: &Command) -> Result<(), Self::Error> {
        let mut trajectories = self.trajectories.lock();
        if trajectories.len() == 2 {
            Err(SearchAgentError::QueueOverflowed)
        } else {
            let trajectory = self.trajectory_generator.generate_search(&command);
            trajectories
                .enqueue(trajectory)
                .unwrap_or_else(|_| unreachable!("This is bug"));
            Ok(())
        }
    }

    fn track_next(&self) -> Result<(), Self::Error> {
        let get_or_init_emergency = || {
            let mut trajectory = self.emergency_trajectory.borrow_mut();
            if trajectory.is_none() {
                trajectory.replace(
                    self.trajectory_generator.generate_emergency(
                        &*self
                            .last_target
                            .borrow()
                            .as_ref()
                            .expect("Should never be None."),
                    ),
                );
                self.emergency_counter.set(0);
            }
            self.emergency_counter.set(self.emergency_counter.get() + 1);
            trajectory.as_mut().expect("Should never be None.").next()
        };

        let (target, is_empty) = {
            if let Some(mut trajectories) = self.trajectories.try_lock() {
                loop {
                    if let Some(trajectory) = trajectories.iter_mut().next() {
                        if let Some(target) = trajectory.nth(self.emergency_counter.get()) {
                            break (Some(target), false);
                        }
                    } else {
                        break (get_or_init_emergency(), true);
                    }
                    trajectories.dequeue();
                }
            } else {
                (get_or_init_emergency(), true)
            }
        };
        if let Some(target) = target.as_ref() {
            self.tracker
                .lock()
                .track(self.state_estimator.borrow().state(), target)?;
        }
        if is_empty {
            Err(SearchAgentError::EmptyTrajectory)
        } else {
            self.last_target.replace(target);
            self.emergency_trajectory.replace(None);
            self.emergency_counter.set(0);
            Ok(())
        }
    }

    fn stop(&self) {
        self.tracker.lock().stop();
    }

    fn correct_state<Diffs: IntoIterator<Item = Diff>>(&self, diffs: Diffs) {
        self.state_estimator.borrow_mut().correct_state(diffs);
    }
}
