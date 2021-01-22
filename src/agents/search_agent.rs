use core::cell::RefCell;

use crate::operators::search_operator::SearchAgent as ISearchAgent;

pub trait TrajectoryManager<Command> {
    type Error;
    type Target;

    fn set_command(&self, command: &Command) -> Result<(), Self::Error>;

    fn next(&self) -> Self::Target;

    fn is_empty(&self) -> Result<bool, Self::Error>;
    fn is_full(&self) -> Result<bool, Self::Error>;
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
            .borrow_mut()
            .track_and_update(&target)
            .map_err(|err| SearchAgentError::Robot(err))
    }

    fn set_command(&self, command: &Command) -> Result<(), Self::Error> {
        self.manager
            .set_command(command)
            .map_err(|err| SearchAgentError::Manager(err))
    }

    fn is_full(&self) -> Result<bool, Self::Error> {
        self.manager
            .is_full()
            .map_err(|err| SearchAgentError::Manager(err))
    }

    fn is_empty(&self) -> Result<bool, Self::Error> {
        self.manager
            .is_empty()
            .map_err(|err| SearchAgentError::Manager(err))
    }
}
