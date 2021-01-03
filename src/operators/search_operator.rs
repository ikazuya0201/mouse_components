use alloc::rc::Rc;
use core::convert::TryInto;
use core::marker::PhantomData;

use crate::administrator::{NotFinishError, Operator};

pub trait SearchAgent<Command> {
    type Error;
    type Obstacle;
    type Obstacles: IntoIterator<Item = Self::Obstacle>;

    fn set_command(&self, command: &Command);
    fn get_obstacles(&self) -> Self::Obstacles;
    //This method is called by interrupt.
    fn track_next(&self) -> Result<(), Self::Error>;
}

pub trait SearchCommander<Obstacle> {
    type Error;
    type Command;

    fn update_obstacles<Obstacles: IntoIterator<Item = Obstacle>>(&self, obstacles: Obstacles);
    fn next_command(&self) -> Result<Self::Command, Self::Error>;
}

pub struct SearchOperator<Mode, Obstacle, Agent, Solver>
where
    Solver: SearchCommander<Obstacle>,
{
    start_command: Solver::Command,
    next_mode: Mode,
    agent: Rc<Agent>,
    solver: Rc<Solver>,
    _obstacle: PhantomData<fn() -> Obstacle>,
}

impl<Mode, Obstacle, Agent, Solver> SearchOperator<Mode, Obstacle, Agent, Solver>
where
    Solver: SearchCommander<Obstacle>,
{
    pub fn new(
        start_command: Solver::Command,
        next_mode: Mode,
        agent: Rc<Agent>,
        solver: Rc<Solver>,
    ) -> Self {
        Self {
            start_command,
            next_mode,
            agent,
            solver,
            _obstacle: PhantomData,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct FinishError;

impl<Mode, Obstacle, Agent, Solver> Operator for SearchOperator<Mode, Obstacle, Agent, Solver>
where
    Mode: Copy,
    Agent: SearchAgent<Solver::Command, Obstacle = Obstacle>,
    Solver: SearchCommander<Obstacle>,
    Solver::Error: TryInto<FinishError>,
{
    type Error = Agent::Error;
    type Mode = Mode;

    fn init(&self) {
        self.agent.set_command(&self.start_command);
    }

    fn tick(&self) -> Result<(), Self::Error> {
        let obstacles = self.agent.get_obstacles();
        self.solver.update_obstacles(obstacles);
        self.agent.track_next()
    }

    fn run(&self) -> Result<Mode, NotFinishError> {
        match self.solver.next_command() {
            Ok(command) => self.agent.set_command(&command),
            Err(err) => match err.try_into() {
                Ok(_) => return Ok(self.next_mode),
                Err(_) => (),
            },
        }
        Err(NotFinishError)
    }
}
