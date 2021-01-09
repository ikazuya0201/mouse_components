use alloc::rc::Rc;
use core::sync::atomic::{AtomicBool, Ordering};

use super::search_operator::CommandConverter;
use crate::administrator::{NotFinishError, Operator};

pub trait RunCommander {
    type Error;
    type Command;
    type Commands: IntoIterator<Item = Self::Command>;

    fn compute_commands(&self) -> Result<Self::Commands, Self::Error>;
}

pub trait RunAgent<Command> {
    type Error;

    fn set_commands<Commands: IntoIterator<Item = Command>>(&self, path: Commands);
    fn track_next(&self) -> Result<(), Self::Error>;
}

pub struct RunOperator<Agent, Commander, Converter> {
    is_completed: AtomicBool,
    agent: Rc<Agent>,
    #[allow(unused)]
    commander: Rc<Commander>,
    #[allow(unused)]
    converter: Converter,
}

impl<Agent, Commander, Converter> RunOperator<Agent, Commander, Converter>
where
    Agent: RunAgent<Converter::Output>,
    Converter: CommandConverter<Commander::Command>,
    Commander: RunCommander,
    Commander::Error: core::fmt::Debug,
{
    pub fn new(agent: Rc<Agent>, commander: Rc<Commander>, converter: Converter) -> Self {
        let commands = commander
            .compute_commands()
            .unwrap_or_else(|err| todo!("Error handling is not implemented: {:?}", err));

        agent.set_commands(
            commands
                .into_iter()
                .map(|command| converter.convert(command)),
        );

        Self {
            is_completed: AtomicBool::new(false),
            agent,
            commander,
            converter,
        }
    }
}

impl<Agent, Commander, Converter> Operator for RunOperator<Agent, Commander, Converter>
where
    Agent: RunAgent<Converter::Output>,
    Converter: CommandConverter<Commander::Command>,
    Commander: RunCommander,
{
    type Error = core::convert::Infallible;

    //TODO: correct agent position by sensor value and wall existence
    fn tick(&self) -> Result<(), Self::Error> {
        if self.agent.track_next().is_err() {
            self.is_completed.store(true, Ordering::Relaxed);
        }
        Ok(())
    }

    fn run(&self) -> Result<(), NotFinishError> {
        if self.is_completed.load(Ordering::Relaxed) {
            Ok(())
        } else {
            Err(NotFinishError)
        }
    }
}
