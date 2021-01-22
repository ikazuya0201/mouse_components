use core::cell::RefCell;

use crate::operators::simple_search_operator::{SearchAgent as ISearchAgent, UpdateError};

//NOTE: Should be implemented as thread-safe.
pub trait TrajectoryManager<Command>: Send + Sync {
    type Target;

    fn set_command(&self, command: &Command);

    //Return the next target and a boolean.
    //The boolean indicates whether the target is regular or not.
    fn next(&self) -> (Self::Target, bool);
}

pub trait Robot<Target> {
    type Error;

    fn track_and_update(&mut self, target: &Target) -> Result<(), Self::Error>;
}

pub struct SearchAgent<Manager, Robot> {
    manager: Manager,
    robot: RefCell<Robot>,
}

impl<Manager, Robot> SearchAgent<Manager, Robot> {
    pub fn new(manager: Manager, robot: Robot) -> Self {
        Self {
            manager,
            robot: RefCell::new(robot),
        }
    }
}

impl<Manager, RobotType, Command> ISearchAgent<Command> for SearchAgent<Manager, RobotType>
where
    RobotType: Robot<Manager::Target>,
    Manager: TrajectoryManager<Command>,
{
    type Error = RobotType::Error;

    fn update(&self) -> Result<(), UpdateError<Self::Error>> {
        let (target, is_regular) = self.manager.next();
        self.robot
            .borrow_mut()
            .track_and_update(&target)
            .map_err(|err| UpdateError::Other(err))?;

        if is_regular {
            Ok(())
        } else {
            Err(UpdateError::EmptyTrajectory)
        }
    }

    fn set_command(&self, command: &Command) -> Result<(), Self::Error> {
        self.manager.set_command(command);
        Ok(())
    }
}
