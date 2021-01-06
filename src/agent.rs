use core::cell::{Cell, RefCell};

use heapless::{spsc::Queue, ArrayLength};
use typenum::consts::*;
use uom::si::f32::{Angle, Length};

use crate::operators::{RunAgent as IRunAgent, SearchAgent as ISearchAgent};
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
    last_target: Cell<Option<Target>>,
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
            last_target: Cell::new(None),
        }
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
where
    ITracker: Tracker<IStateEstimator::State, Target>,
    IStateEstimator: StateEstimator,
{
    pub fn stop(&self) {
        self.tracker.borrow_mut().stop();
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
    ITrajectoryGenerator::Target: Copy,
    IObstacleDetector: ObstacleDetector<IStateEstimator::State>,
    IStateEstimator: StateEstimator,
    ITracker: Tracker<IStateEstimator::State, ITrajectoryGenerator::Target>,
    ITrajectoryGenerator: SearchTrajectoryGenerator<Pose, Kind>,
{
    type Error = ITracker::Error;
    type Obstacle = IObstacleDetector::Obstacle;
    type Obstacles = IObstacleDetector::Obstacles;

    fn update_state(&self) {
        self.state_estimator.borrow_mut().estimate();
    }

    fn get_obstacles(&self) -> Self::Obstacles {
        let state = self.state_estimator.borrow().state();
        self.obstacle_detector.borrow_mut().detect(&state)
    }

    fn set_command(&self, command: &(Pose, Kind)) {
        let trajectory = self
            .trajectory_generator
            .generate_search(&command.0, &command.1);
        let mut trajectories = self.trajectories.lock();
        trajectories.enqueue(trajectory).ok();
    }

    fn track_next(&self) -> Result<(), Self::Error> {
        let state = self.state_estimator.borrow().state();
        let target = {
            if let Ok(mut trajectories) = self.trajectories.try_lock() {
                loop {
                    if let Some(trajectory) = trajectories.iter_mut().next() {
                        if let Some(target) = trajectory.next() {
                            break Some(target);
                        }
                    } else {
                        break self.last_target.get();
                    }
                    trajectories.dequeue();
                }
            } else {
                self.last_target.get()
            }
        };
        if let Some(target) = target {
            self.tracker.borrow_mut().track(&state, &target)?;
            self.last_target.set(Some(target));
        }
        Ok(())
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
