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

pub struct SearchOperator<Obstacle, Command, Agent, Commander, Converter> {
    keeped_command: RefCell<Option<Command>>,
    agent: Agent,
    commander: Commander,
    converter: Converter,
    _obstacle: PhantomData<fn() -> Obstacle>,
}

impl<Obstacle, Command, Agent, Commander, Converter>
    SearchOperator<Obstacle, Command, Agent, Commander, Converter>
{
    pub fn consume(self) -> (Agent, Commander, Converter) {
        let SearchOperator {
            keeped_command: _,
            agent,
            commander,
            converter,
            _obstacle,
        } = self;
        (agent, commander, converter)
    }
}

impl<Obstacle, Command, Agent, Commander, Converter>
    SearchOperator<Obstacle, Command, Agent, Commander, Converter>
where
    Agent: SearchAgent<Converter::Output, Obstacle = Obstacle>,
    Converter: CommandConverter<Commander::Command>,
    Commander: SearchCommander<Obstacle>,
{
    pub fn new(agent: Agent, commander: Commander, converter: Converter) -> Self {
        let command = converter.convert(
            commander
                .next_command()
                .unwrap_or_else(|_| unimplemented!("This error handling is not implemented.")),
        );
        agent
            .set_command(&command)
            .unwrap_or_else(|_| unreachable!());

        Self {
            keeped_command: RefCell::new(None),
            agent,
            commander,
            converter,
            _obstacle: PhantomData,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct FinishError;

impl<Obstacle, Agent, Commander, Converter> Operator
    for SearchOperator<Obstacle, Converter::Output, Agent, Commander, Converter>
where
    Agent: SearchAgent<Converter::Output, Obstacle = Obstacle>,
    Converter: CommandConverter<Commander::Command>,
    Commander: SearchCommander<Obstacle>,
    Commander::Error: TryInto<FinishError>,
{
    type Error = Agent::Error;

    fn tick(&self) -> Result<(), Self::Error> {
        self.agent.update_state();
        let obstacles = self.agent.get_obstacles();
        self.commander.update_obstacles(obstacles);
        self.agent.track_next()
    }

    fn run(&self) -> Result<(), NotFinishError> {
        let mut keeped_command = self.keeped_command.borrow_mut();
        if let Some(command) = keeped_command.as_ref() {
            if self.agent.set_command(command).is_ok() {
                keeped_command.take();
            }
        } else {
            match self.commander.next_command() {
                Ok(command) => {
                    let command = self.converter.convert(command);
                    if self.agent.set_command(&command).is_err() {
                        keeped_command.replace(command);
                    }
                }
                Err(err) => {
                    if err.try_into().is_ok() {
                        return Ok(());
                    }
                }
            }
        }
        Err(NotFinishError)
    }
}
