use spin::Mutex;

use crate::agents::Robot;
use crate::operators::TrackingAgent as ITrackingAgent;
use crate::{Construct, Deconstruct, Merge};

/// A trait that manages trajectories.
pub trait TrajectoryManager<Command> {
    type Error;
    type Target;

    fn set_command(&self, command: &Command) -> Result<(), Self::Error>;

    fn next(&self) -> Self::Target;

    fn is_empty(&self) -> Option<bool>;
    fn is_full(&self) -> Option<bool>;
}

/// An implementation of [TrackingAgent](crate::operators::TrackingAgent) required by
/// [TrackingOperator](crate::operators::TrackingOperator).
pub struct TrackingAgent<Manager, Robot> {
    manager: Manager,
    robot: Mutex<Robot>,
}

impl<Manager, Robot> TrackingAgent<Manager, Robot> {
    pub fn new(manager: Manager, robot: Robot) -> Self {
        Self {
            manager,
            robot: Mutex::new(robot),
        }
    }

    pub fn release(self) -> (Manager, Robot) {
        let Self { manager, robot } = self;
        (manager, robot.into_inner())
    }
}

impl<Manager, Robot, Config, State, Resource> Construct<Config, State, Resource>
    for TrackingAgent<Manager, Robot>
where
    Manager: Construct<Config, State, Resource>,
    Robot: Construct<Config, State, Resource>,
{
    fn construct(config: &Config, state: &State, resource: Resource) -> (Self, Resource) {
        let (manager, resource) = Manager::construct(config, state, resource);
        let (robot, resource) = Robot::construct(config, state, resource);
        (Self::new(manager, robot), resource)
    }
}

impl<Manager, Robot, State, Resource> Deconstruct<State, Resource> for TrackingAgent<Manager, Robot>
where
    Manager: Deconstruct<State, Resource>,
    Robot: Deconstruct<State, Resource>,
    State: Merge,
    Resource: Merge,
{
    fn deconstruct(self) -> (State, Resource) {
        let (manager, robot) = self.release();
        let (manager_state, manager_resource) = manager.deconstruct();
        let (robot_state, robot_resource) = robot.deconstruct();
        (
            manager_state.merge(robot_state),
            manager_resource.merge(robot_resource),
        )
    }
}

/// Error on [TrackingAgent](TrackingAgent).
#[derive(Debug)]
pub enum TrackingAgentError<T, U> {
    Robot(T),
    Manager(U),
}

impl<Manager, RobotType, Command> ITrackingAgent<Command> for TrackingAgent<Manager, RobotType>
where
    RobotType: Robot<Manager::Target>,
    Manager: TrajectoryManager<Command>,
{
    type Error = TrackingAgentError<RobotType::Error, Manager::Error>;

    fn update(&self) -> Result<(), Self::Error> {
        let target = self.manager.next();
        self.robot
            .lock()
            .track_and_update(&target)
            .map_err(|err| TrackingAgentError::Robot(err))
    }

    fn set_command(&self, command: &Command) -> Result<(), Self::Error> {
        self.manager
            .set_command(command)
            .map_err(|err| TrackingAgentError::Manager(err))
    }

    fn is_full(&self) -> Option<bool> {
        self.manager.is_full()
    }

    fn is_empty(&self) -> Option<bool> {
        self.manager.is_empty()
    }
}
