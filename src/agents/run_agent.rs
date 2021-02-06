use spin::Mutex;

use crate::agents::Robot;
use crate::operators::RunAgent as IRunAgent;

pub trait TrajectoryManager<Command> {
    type Target;

    fn set_commands<Commands: IntoIterator<Item = Command>>(&self, commands: Commands);
    fn next(&self) -> Option<Self::Target>;
}

pub struct RunAgent<Manager, Robot> {
    manager: Manager,
    robot: Mutex<Robot>,
}

impl<Manager, Robot> RunAgent<Manager, Robot> {
    pub fn new(manager: Manager, robot: Robot) -> Self {
        Self {
            manager,
            robot: Mutex::new(robot),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RunAgentError<T> {
    Manager,
    Robot(T),
}

impl<Command, Manager, RobotType> IRunAgent<Command> for RunAgent<Manager, RobotType>
where
    Manager: TrajectoryManager<Command>,
    RobotType: Robot<Manager::Target>,
{
    type Error = RunAgentError<RobotType::Error>;

    fn set_commands<Commands: IntoIterator<Item = Command>>(&self, commands: Commands) {
        self.manager.set_commands(commands);
    }

    fn track_next(&self) -> Result<(), Self::Error> {
        let target = self.manager.next().ok_or(RunAgentError::Manager)?;
        self.robot
            .lock()
            .track_and_update(&target)
            .map_err(|err| RunAgentError::Robot(err))
    }
}
