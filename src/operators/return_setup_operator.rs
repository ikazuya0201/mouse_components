use core::sync::atomic::{AtomicBool, Ordering};

use crate::administrator::{IncompletedError, Operator};

/// An implementation of [Operator](crate::administrator::Operator) required by
/// [Administrator](crate::administrator::Administrator).
///
/// This operates a process that sets up an initial position for next
/// [RunOperator](crate::operators::RunOperator).
pub struct ReturnSetupOperator<Agent> {
    is_completed: AtomicBool,
    agent: Agent,
}

/// A trait that computes a command for setting up return to start.
pub trait ReturnSetupCommander {
    type Error;
    type Command;

    fn setup_command(&self) -> Result<Self::Command, Self::Error>;
}

/// A trait that consume a command for setting up return to start.
pub trait ReturnSetupCommandConsumer<Command> {
    type Error;

    fn set_command(&self, command: &Command) -> Result<(), Self::Error>;
}

/// Error on [ReturnSetupAgent](ReturnSetupAgent).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ReturnSetupAgentError<T> {
    Completed,
    Other(T),
}

/// A trait that processes tracking.
pub trait ReturnSetupAgent {
    type Error;

    fn track_next(&self) -> Result<(), ReturnSetupAgentError<Self::Error>>;
}

impl<Agent> ReturnSetupOperator<Agent> {
    pub fn new<Commander>(agent: Agent, commander: Commander) -> Self
    where
        Agent: ReturnSetupCommandConsumer<Commander::Command>,
        Commander: ReturnSetupCommander,
        Agent::Error: core::fmt::Debug,
        Commander::Error: core::fmt::Debug,
    {
        let command = commander
            .setup_command()
            .unwrap_or_else(|err| unreachable!("Should never fail: {:?}", err));
        agent
            .set_command(&command)
            .unwrap_or_else(|err| unreachable!("Should never fail: {:?}", err));
        Self {
            agent,
            is_completed: AtomicBool::new(false),
        }
    }
}

impl<Agent> Operator for ReturnSetupOperator<Agent>
where
    Agent: ReturnSetupAgent,
{
    type Error = Agent::Error;

    fn tick(&self) -> Result<(), Self::Error> {
        match self.agent.track_next() {
            Ok(()) => Ok(()),
            Err(err) => match err {
                ReturnSetupAgentError::Completed => {
                    self.is_completed.store(true, Ordering::Release);
                    Ok(())
                }
                ReturnSetupAgentError::Other(err) => Err(err),
            },
        }
    }

    fn run(&self) -> Result<(), Result<IncompletedError, Self::Error>> {
        if self.is_completed.load(Ordering::Acquire) {
            Err(Ok(IncompletedError))
        } else {
            Ok(())
        }
    }
}
