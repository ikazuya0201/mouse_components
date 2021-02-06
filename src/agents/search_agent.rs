use spin::Mutex;

use crate::agents::Robot;
use crate::operators::SearchAgent as ISearchAgent;

/// A trait that manages trajectories.
pub trait TrajectoryManager<Command> {
    type Error;
    type Target;

    fn set_command(&self, command: &Command) -> Result<(), Self::Error>;

    fn next(&self) -> Self::Target;

    fn is_empty(&self) -> Option<bool>;
    fn is_full(&self) -> Option<bool>;
}

/// An implementation of [SearchAgent](crate::operators::SearchAgent) required by
/// [SearchOperator](crate::operators::SearchOperator).
pub struct SearchAgent<Manager, Robot> {
    manager: Manager,
    robot: Mutex<Robot>,
}

impl<Manager, Robot> SearchAgent<Manager, Robot> {
    pub fn new(manager: Manager, robot: Robot) -> Self {
        Self {
            manager,
            robot: Mutex::new(robot),
        }
    }
}

#[derive(Debug)]
pub enum SearchAgentError<T, U> {
    Robot(T),
    Manager(U),
}

impl<Manager, RobotType, Command> ISearchAgent<Command> for SearchAgent<Manager, RobotType>
where
    RobotType: Robot<Manager::Target>,
    Manager: TrajectoryManager<Command>,
{
    type Error = SearchAgentError<RobotType::Error, Manager::Error>;

    fn update(&self) -> Result<(), Self::Error> {
        let target = self.manager.next();
        self.robot
            .lock()
            .track_and_update(&target)
            .map_err(|err| SearchAgentError::Robot(err))
    }

    fn set_command(&self, command: &Command) -> Result<(), Self::Error> {
        self.manager
            .set_command(command)
            .map_err(|err| SearchAgentError::Manager(err))
    }

    fn is_full(&self) -> Option<bool> {
        self.manager.is_full()
    }

    fn is_empty(&self) -> Option<bool> {
        self.manager.is_empty()
    }
}
