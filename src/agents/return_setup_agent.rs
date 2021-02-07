use spin::Mutex;

use super::Robot as IRobot;
use crate::operators::{
    ReturnSetupAgent as IReturnSetupAgent, ReturnSetupAgentError, ReturnSetupCommandConsumer,
};

/// A trait that manages trajectory.
pub trait TrajectoryManager {
    type Target;

    fn next(&self) -> Option<Self::Target>;
}

/// An implementation of [ReturnSetupAgent](crate::operators::ReturnSetupAgent).
pub struct ReturnSetupAgent<Manager, Robot> {
    manager: Manager,
    robot: Mutex<Robot>,
}

impl<Manager, Robot> ReturnSetupAgent<Manager, Robot> {
    pub fn new(manager: Manager, robot: Robot) -> Self {
        Self {
            manager,
            robot: Mutex::new(robot),
        }
    }
}

impl<Manager, Robot> IReturnSetupAgent for ReturnSetupAgent<Manager, Robot>
where
    Manager: TrajectoryManager,
    Robot: IRobot<Manager::Target>,
{
    type Error = Robot::Error;

    fn track_next(&self) -> Result<(), ReturnSetupAgentError<Self::Error>> {
        let target = self
            .manager
            .next()
            .ok_or(ReturnSetupAgentError::Completed)?;
        self.robot
            .lock()
            .track_and_update(&target)
            .map_err(|err| ReturnSetupAgentError::Other(err))
    }
}

impl<Manager, Robot, Command> ReturnSetupCommandConsumer<Command>
    for ReturnSetupAgent<Manager, Robot>
where
    Manager: ReturnSetupCommandConsumer<Command>,
{
    type Error = Manager::Error;

    fn set_command(&self, command: &Command) -> Result<(), Self::Error> {
        self.manager.set_command(command)
    }
}
