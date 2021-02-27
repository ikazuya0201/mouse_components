use spin::Mutex;

use crate::agents::Robot as IRobot;
use crate::operators::{TrackingAgent as ITrackingAgent, TrackingAgentError, TrackingInitializer};

/// A trait that produces tracking target.
pub trait TrajectoryManager {
    type Target;

    fn next(&self) -> Option<Self::Target>;
}

/// An implementation of [TrackingAgent](crate::operators::TrackingAgent).
#[derive(Debug)]
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
}

impl<'a, Resource, Config, State, Manager, Robot> From<(Resource, &'a Config, &'a State)>
    for TrackingAgent<Manager, Robot>
where
    Manager: From<(&'a Config, &'a State)>,
    Robot: From<(Resource, &'a Config, &'a State)>,
{
    fn from((resource, config, state): (Resource, &'a Config, &'a State)) -> Self {
        Self::new(
            Manager::from((config, state)),
            Robot::from((resource, config, state)),
        )
    }
}

impl<Command, Manager, Robot> TrackingInitializer<Command> for TrackingAgent<Manager, Robot>
where
    Manager: TrackingInitializer<Command>,
{
    type Error = Manager::Error;

    fn initialize<Commands: IntoIterator<Item = Command>>(
        &self,
        commands: Commands,
    ) -> Result<(), Self::Error> {
        self.manager.initialize(commands)
    }
}

impl<Manager, Robot> ITrackingAgent for TrackingAgent<Manager, Robot>
where
    Manager: TrajectoryManager,
    Robot: IRobot<Manager::Target>,
{
    type Error = Robot::Error;

    fn track_next(&self) -> Result<(), TrackingAgentError<Self::Error>> {
        let target = self.manager.next().ok_or(TrackingAgentError::Completed)?;
        self.robot
            .lock()
            .track_and_update(&target)
            .map_err(|err| TrackingAgentError::Other(err))
    }
}
