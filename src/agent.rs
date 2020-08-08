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
    fn estimate(&mut self) -> State;
}

pub trait TrajectoryGenerator<Pose, Target, Direction> {
    type Trajectory: Iterator<Item = Target>;

    fn generate_search(&self, pose: Pose, direction: Direction) -> Self::Trajectory;
}

pub trait Tracker<State, Target> {
    fn track(&mut self, state: State, target: Target);
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
    _position: PhantomData<fn() -> Obstacle>,
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
