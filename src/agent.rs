use core::cell::{Cell, RefCell};

use heapless::{spsc::Queue, ArrayLength};
use typenum::consts::*;
use uom::si::f32::{Angle, Length};

use crate::operators::{EmptyTrajectoyError, RunAgent as IRunAgent, SearchAgent as ISearchAgent};
use crate::utils::mutex::Mutex;

pub trait ObstacleDetector<State> {
    type Obstacle;
    type Obstacles: IntoIterator<Item = Self::Obstacle>;

    fn detect(&mut self, state: &State) -> Self::Obstacles;
}

pub trait StateEstimator {
    type State;

    fn init(&mut self);
    fn estimate(&mut self);
    fn state(&self) -> Self::State;
}

pub trait SearchTrajectoryGenerator<Pose, Kind> {
    type Target;
    type Trajectory: Iterator<Item = Self::Target>;

    fn generate_search(&self, pose: &Pose, kind: &Kind) -> Self::Trajectory;
    fn generate_emergency(&self, target: &Self::Target) -> Self::Trajectory;
}

pub trait Tracker<State, Target> {
    type Error;

    fn init(&mut self);
    fn track(&mut self, state: &State, target: &Target) -> Result<(), Self::Error>;
    fn stop(&mut self);
}

pub trait RunTrajectoryGenerator<Command> {
    type Target;
    type Trajectory: Iterator<Item = Self::Target>;
    type Trajectories: IntoIterator<Item = Self::Trajectory>;

    fn generate<Commands: IntoIterator<Item = Command>>(
        &self,
        commands: Commands,
    ) -> Self::Trajectories;
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Pose {
    pub x: Length,
    pub y: Length,
    pub theta: Angle,
}

impl Pose {
    pub fn new(x: Length, y: Length, theta: Angle) -> Self {
        Self { x, y, theta }
    }
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
    tracker: RefCell<ITracker>,
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
            tracker: RefCell::new(tracker),
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

impl<Pose, Kind, IObstacleDetector, IStateEstimator, ITracker, ITrajectoryGenerator>
    ISearchAgent<(Pose, Kind)>
    for SearchAgent<
        IObstacleDetector,
        IStateEstimator,
        ITracker,
        ITrajectoryGenerator,
        ITrajectoryGenerator::Trajectory,
        ITrajectoryGenerator::Target,
    >
where
    Pose: Copy,
    IObstacleDetector: ObstacleDetector<IStateEstimator::State>,
    IStateEstimator: StateEstimator,
    ITracker: Tracker<IStateEstimator::State, ITrajectoryGenerator::Target>,
    ITrajectoryGenerator: SearchTrajectoryGenerator<Pose, Kind>,
{
    type Error = SearchAgentError<ITracker::Error>;
    type Obstacle = IObstacleDetector::Obstacle;
    type Obstacles = IObstacleDetector::Obstacles;

    fn update_state(&self) {
        self.state_estimator.borrow_mut().estimate();
    }

    fn get_obstacles(&self) -> Self::Obstacles {
        let state = self.state_estimator.borrow().state();
        self.obstacle_detector.borrow_mut().detect(&state)
    }

    fn set_command(&self, command: &(Pose, Kind)) -> Result<(), Self::Error> {
        let mut trajectories = self.trajectories.lock();
        if trajectories.len() == 2 {
            Err(SearchAgentError::QueueOverflowed)
        } else {
            let trajectory = self
                .trajectory_generator
                .generate_search(&command.0, &command.1);
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

        let state = self.state_estimator.borrow().state();
        let (target, is_empty) = {
            if let Ok(mut trajectories) = self.trajectories.try_lock() {
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
            self.tracker.borrow_mut().track(&state, target)?;
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
    StateEstimatorType: StateEstimator,
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
