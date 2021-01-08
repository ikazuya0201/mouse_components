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

pub struct RunOperator<Mode, Agent, Commander, Converter> {
    next_mode: Mode,
    is_completed: AtomicBool,
    agent: Rc<Agent>,
    commander: Rc<Commander>,
    converter: Converter,
}

impl<Mode, Agent, Commander, Converter> RunOperator<Mode, Agent, Commander, Converter> {
    pub fn new(
        next_mode: Mode,
        agent: Rc<Agent>,
        commander: Rc<Commander>,
        converter: Converter,
    ) -> Self {
        Self {
            next_mode,
            is_completed: AtomicBool::new(false),
            agent,
            commander,
            converter,
        }
    }
}

impl<Mode, Agent, Commander, Converter> Operator for RunOperator<Mode, Agent, Commander, Converter>
where
    Mode: Copy,
    Agent: RunAgent<Converter::Output>,
    Converter: CommandConverter<Commander::Command>,
    Commander: RunCommander,
    Commander::Error: core::fmt::Debug,
{
    type Error = core::convert::Infallible;
    type Mode = Mode;

    fn init(&self) {
        //TODO: modify to return Result in init. remove this unwrap.
        let commands = self
            .commander
            .compute_commands()
            .unwrap_or_else(|err| todo!("Error handling is not implemented: {:?}", err));

        self.agent.set_commands(
            commands
                .into_iter()
                .map(|command| self.converter.convert(command)),
        );
    }

    //TODO: correct agent position by sensor value and wall existence
    fn tick(&self) -> Result<(), Self::Error> {
        if self.agent.track_next().is_err() {
            self.is_completed.store(true, Ordering::Relaxed);
        }
        Ok(())
    }

    fn run(&self) -> Result<Mode, NotFinishError> {
        if self.is_completed.load(Ordering::Relaxed) {
            Ok(self.next_mode)
        } else {
            Err(NotFinishError)
        }
    }
}
