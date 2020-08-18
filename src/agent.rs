pub mod pose;

use core::cell::RefCell;
use core::marker::PhantomData;

use heapless::{consts::*, spsc::Queue};

use super::administrator;
use crate::utils::mutex::Mutex;
pub use pose::Pose;

pub trait ObstacleDetector<Obstacle, State> {
    type Obstacles: IntoIterator<Item = Obstacle>;

    fn detect(&mut self, state: State) -> Self::Obstacles;
}

pub trait StateEstimator<State> {
    fn init(&mut self);
    fn estimate(&mut self) -> State;
}

pub trait TrajectoryGenerator<Pose, Target, Direction> {
    type Trajectory: Iterator<Item = Target>;

    fn generate_search_init(&self, pose: Pose) -> Self::Trajectory;
    fn generate_search(&self, pose: Pose, direction: Direction) -> Self::Trajectory;
}

pub trait Tracker<State, Target> {
    fn init(&mut self);
    fn track(&mut self, state: State, target: Target);
    fn stop(&mut self);
}

pub struct Agent<
    State,
    Target,
    Pose,
    Obstacle,
    Direction,
    IObstacleDetector,
    IStateEstimator,
    ITracker,
    ITrajectoryGenerator,
> where
    IObstacleDetector: ObstacleDetector<Obstacle, State>,
    IStateEstimator: StateEstimator<State>,
    ITracker: Tracker<State, Target>,
    ITrajectoryGenerator: TrajectoryGenerator<Pose, Target, Direction>,
{
    obstacle_detector: RefCell<IObstacleDetector>,
    state_estimator: RefCell<IStateEstimator>,
    tracker: RefCell<ITracker>,
    trajectory_generator: ITrajectoryGenerator,
    trajectories: Mutex<Queue<ITrajectoryGenerator::Trajectory, U3>>,
    _state: PhantomData<fn() -> State>,
    _target: PhantomData<fn() -> Target>,
    _pose: PhantomData<fn() -> Pose>,
    _obstacle: PhantomData<fn() -> Obstacle>,
    _direction: PhantomData<fn() -> Direction>,
}

impl<
        State,
        Target,
        Pose,
        Obstacle,
        Direction,
        IObstacleDetector,
        IStateEstimator,
        ITracker,
        ITrajectoryGenerator,
    >
    Agent<
        State,
        Target,
        Pose,
        Obstacle,
        Direction,
        IObstacleDetector,
        IStateEstimator,
        ITracker,
        ITrajectoryGenerator,
    >
where
    IObstacleDetector: ObstacleDetector<Obstacle, State>,
    IStateEstimator: StateEstimator<State>,
    ITracker: Tracker<State, Target>,
    ITrajectoryGenerator: TrajectoryGenerator<Pose, Target, Direction>,
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
            _state: PhantomData,
            _target: PhantomData,
            _pose: PhantomData,
            _obstacle: PhantomData,
            _direction: PhantomData,
        }
    }

    pub fn stop(&self) {
        self.tracker.borrow_mut().stop();
    }
}

impl<
        State,
        Target,
        Pose,
        Obstacle,
        Direction,
        IObstacleDetector,
        IStateEstimator,
        ITracker,
        ITrajectoryGenerator,
    > administrator::Agent<Obstacle, Pose, Direction>
    for Agent<
        State,
        Target,
        Pose,
        Obstacle,
        Direction,
        IObstacleDetector,
        IStateEstimator,
        ITracker,
        ITrajectoryGenerator,
    >
where
    Pose: Copy,
    IObstacleDetector: ObstacleDetector<Obstacle, State>,
    IStateEstimator: StateEstimator<State>,
    ITracker: Tracker<State, Target>,
    ITrajectoryGenerator: TrajectoryGenerator<Pose, Target, Direction>,
{
    type Obstacles = IObstacleDetector::Obstacles;

    fn init(&self, pose: Pose) {
        self.state_estimator.borrow_mut().init();
        self.tracker.borrow_mut().init();
        let trajectory = self.trajectory_generator.generate_search_init(pose);
        let mut trajectories = self.trajectories.lock();
        *trajectories = Queue::new();
        trajectories.enqueue(trajectory).ok();
    }

    fn get_existing_obstacles(&self) -> Self::Obstacles {
        let state = self.state_estimator.borrow_mut().estimate();
        self.obstacle_detector.borrow_mut().detect(state)
    }

    fn set_instructed_direction(&self, pose: Pose, direction: Direction) {
        let trajectory = self.trajectory_generator.generate_search(pose, direction);
        let mut trajectories = self.trajectories.lock();
        trajectories.enqueue(trajectory).ok();
    }

    fn track_next(&self) {
        let state = self.state_estimator.borrow_mut().estimate();
        while let Ok(mut trajectories) = self.trajectories.try_lock() {
            if let Some(trajectory) = trajectories.iter_mut().next() {
                if let Some(target) = trajectory.next() {
                    self.tracker.borrow_mut().track(state, target);
                    return;
                }
            } else {
                return;
            }
            trajectories.dequeue();
        }
    }
}
