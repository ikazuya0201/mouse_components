use alloc::rc::Rc;
use core::cell::RefCell;
use core::convert::TryInto;
use core::marker::PhantomData;

use crate::administrator::{NotFinishError, Operator};

pub trait SearchAgent<Command> {
    type Error;
    type Obstacle;
    type Obstacles: IntoIterator<Item = Self::Obstacle>;

    //update estimated state
    fn update_state(&self);
    fn set_command(&self, command: &Command) -> Result<(), Self::Error>;
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

pub trait CommandConverter<Command> {
    type Output;

    fn convert(&self, source: Command) -> Self::Output;
}

pub struct SearchOperator<Mode, Obstacle, Command, Agent, Solver, Converter> {
    keeped_command: RefCell<Option<Command>>,
    next_mode: Mode,
    agent: Rc<Agent>,
    solver: Rc<Solver>,
    converter: Converter,
    _obstacle: PhantomData<fn() -> Obstacle>,
}

impl<Mode, Obstacle, Command, Agent, Solver, Converter>
    SearchOperator<Mode, Obstacle, Command, Agent, Solver, Converter>
{
    pub fn new(
        next_mode: Mode,
        agent: Rc<Agent>,
        solver: Rc<Solver>,
        converter: Converter,
    ) -> Self {
        Self {
            keeped_command: RefCell::new(None),
            next_mode,
            agent,
            solver,
            converter,
            _obstacle: PhantomData,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct FinishError;

impl<Mode, Obstacle, Agent, Solver, Converter> Operator
    for SearchOperator<Mode, Obstacle, Converter::Output, Agent, Solver, Converter>
where
    Mode: Copy,
    Agent: SearchAgent<Converter::Output, Obstacle = Obstacle>,
    Converter: CommandConverter<Solver::Command>,
    Solver: SearchCommander<Obstacle>,
    Solver::Error: TryInto<FinishError>,
{
    type Error = Agent::Error;
    type Mode = Mode;

    fn init(&self) {
        let command = self.converter.convert(
            self.solver
                .next_command()
                .unwrap_or_else(|_| unimplemented!("This error handling is not implemented.")),
        );
        self.agent
            .set_command(&command)
            .unwrap_or_else(|_| unreachable!());
    }

    fn tick(&self) -> Result<(), Self::Error> {
        self.agent.update_state();
        let obstacles = self.agent.get_obstacles();
        self.solver.update_obstacles(obstacles);
        self.agent.track_next()
    }

    fn run(&self) -> Result<Mode, NotFinishError> {
        let mut keeped_command = self.keeped_command.borrow_mut();
        if let Some(command) = keeped_command.as_ref() {
            if self.agent.set_command(command).is_ok() {
                keeped_command.take();
            }
        } else {
            match self.solver.next_command() {
                Ok(command) => {
                    let command = self.converter.convert(command);
                    if self.agent.set_command(&command).is_err() {
                        keeped_command.replace(command);
                    }
                }
                Err(err) => {
                    if err.try_into().is_ok() {
                        return Ok(self.next_mode);
                    }
                }
            }
        }
        Err(NotFinishError)
    }
}
