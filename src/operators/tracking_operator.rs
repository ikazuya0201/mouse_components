use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::administrator::{Operator, OperatorError};

/// An implementation of [Operator](crate::administrator::Operator) required by
/// [Administrator](crate::administrator::Administrator).
///
/// This operator executes a tracking process.
/// The commands for tracking are given once at the initialization phase.
#[derive(Debug)]
pub struct TrackingOperator<Commander, Agent> {
    is_completed: AtomicBool,
    agent: Agent,
    _commander: PhantomData<fn() -> Commander>,
}

impl<'a, CResource, AResource, State, Config, Commander, Agent>
    From<((CResource, AResource), &'a State, &'a Config)> for TrackingOperator<Commander, Agent>
where
    Commander: From<(CResource, &'a State, &'a Config)>,
    Agent: From<(AResource, &'a State, &'a Config)>,
    Agent: TrackingInitializer<Commander::Command>,
    Commander: InitialCommander,
    Agent::Error: core::fmt::Debug,
    Commander::Error: core::fmt::Debug,
{
    fn from(
        ((commander, agent), state, config): ((CResource, AResource), &'a State, &'a Config),
    ) -> Self {
        Self::new(
            Agent::from((agent, state, config)),
            Commander::from((commander, state, config)),
        )
    }
}

/// A trait that generates initial commands.
pub trait InitialCommander {
    type Error;
    type Command;
    type Commands: IntoIterator<Item = Self::Command>;

    fn initial_commands(&self) -> Result<Self::Commands, Self::Error>;
}

/// A trait that is initialized by given commands.
pub trait TrackingInitializer<Command> {
    type Error;

    fn initialize<Commands: IntoIterator<Item = Command>>(
        &self,
        commands: Commands,
    ) -> Result<(), Self::Error>;
}

/// Error on [TrackingAgent](TrackingAgent).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TrackingAgentError<T> {
    Completed,
    Other(T),
}

/// A trait that processes tracking.
pub trait TrackingAgent {
    type Error;

    fn track_next(&self) -> Result<(), TrackingAgentError<Self::Error>>;
}

impl<Commander, Agent> TrackingOperator<Commander, Agent> {
    pub fn new(agent: Agent, commander: Commander) -> Self
    where
        Agent: TrackingInitializer<Commander::Command>,
        Commander: InitialCommander,
        Agent::Error: core::fmt::Debug,
        Commander::Error: core::fmt::Debug,
    {
        let commands = commander
            .initial_commands()
            .unwrap_or_else(|err| unreachable!("Should never fail: {:?}", err));
        agent
            .initialize(commands)
            .unwrap_or_else(|err| unreachable!("Should never fail: {:?}", err));
        Self {
            agent,
            is_completed: AtomicBool::new(false),
            _commander: PhantomData,
        }
    }
}

impl<Commander, Agent> Operator for TrackingOperator<Commander, Agent>
where
    Agent: TrackingAgent,
{
    type Error = Agent::Error;

    fn tick(&self) -> Result<(), Self::Error> {
        match self.agent.track_next() {
            Ok(()) => Ok(()),
            Err(err) => match err {
                TrackingAgentError::Completed => {
                    self.is_completed.store(true, Ordering::Release);
                    Ok(())
                }
                TrackingAgentError::Other(err) => Err(err),
            },
        }
    }

    fn run(&self) -> Result<(), OperatorError<Self::Error>> {
        if self.is_completed.load(Ordering::Acquire) {
            Ok(())
        } else {
            Err(OperatorError::Incompleted)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tracking_operator() {
        use core::cell::RefCell;

        struct Agent {
            command: RefCell<Option<usize>>,
            state: RefCell<usize>,
        }

        impl TrackingInitializer<usize> for Agent {
            type Error = core::convert::Infallible;

            fn initialize<Commands: IntoIterator<Item = usize>>(
                &self,
                commands: Commands,
            ) -> Result<(), Self::Error> {
                self.command.replace(commands.into_iter().next());
                Ok(())
            }
        }

        impl TrackingAgent for Agent {
            type Error = core::convert::Infallible;

            fn track_next(&self) -> Result<(), TrackingAgentError<Self::Error>> {
                let mut state = self.state.borrow_mut();
                if *state >= 2 {
                    Err(TrackingAgentError::Completed)
                } else {
                    *state += 1;
                    Ok(())
                }
            }
        }

        struct Commander;

        impl InitialCommander for Commander {
            type Error = core::convert::Infallible;
            type Command = usize;
            type Commands = Vec<usize>;

            fn initial_commands(&self) -> Result<Self::Commands, Self::Error> {
                Ok(vec![1])
            }
        }

        let operator = TrackingOperator::new(
            Agent {
                command: RefCell::new(None),
                state: RefCell::new(0),
            },
            Commander,
        );

        assert_eq!(operator.agent.command, RefCell::new(Some(1)));

        assert_eq!(operator.tick(), Ok(()));
        assert_eq!(operator.run(), Err(OperatorError::Incompleted));
        assert_eq!(operator.tick(), Ok(()));
        assert_eq!(operator.run(), Err(OperatorError::Incompleted));
        assert_eq!(operator.tick(), Ok(()));
        assert_eq!(operator.run(), Ok(()));
    }
}
