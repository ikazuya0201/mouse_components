use crate::administrator::{IncompletedError, Operator};

/// A trait that processes command and moves the machine.
///
/// Physical quantities should be maanged only in this trait.
pub trait SearchAgent<Command> {
    type Error;

    fn update(&self) -> Result<(), Self::Error>;
    fn set_command(&self, command: &Command) -> Result<(), Self::Error>;
    fn is_full(&self) -> Option<bool>;
    fn is_empty(&self) -> Option<bool>;
}

/// Error on [SearchCommander](SearchCommander)
///
/// - SearchFinish indicates the search process has completed.
/// - Waiting indicates the calculation process of next command is being blocked now.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SearchCommanderError<T> {
    SearchFinish,
    Waiting,
    Other(T),
}

/// A trait that calculates and instructs commands for agent.
pub trait SearchCommander {
    type Error;
    type Command;

    fn next_command(&self) -> Result<Self::Command, SearchCommanderError<Self::Error>>;
}

/// An implementation of [Operator](crate::administrator::Operator) required by
/// [Administrator](crate::administrator::Administrator).
///
/// This processes a search of a maze.
pub struct SearchOperator<Commander, Agent> {
    commander: Commander,
    agent: Agent,
}

impl<Commander, Agent> SearchOperator<Commander, Agent>
where
    Commander: SearchCommander,
    Agent: SearchAgent<Commander::Command>,
{
    pub fn new(commander: Commander, agent: Agent) -> Self {
        let command = commander.next_command().unwrap_or_else(|_| unreachable!());
        agent
            .set_command(&command)
            .unwrap_or_else(|_| unreachable!());
        Self { commander, agent }
    }
}

#[derive(Debug)]
pub enum SearchOperatorError<T, U> {
    Agent(T),
    Commander(U),
}

impl<Commander, Agent> Operator for SearchOperator<Commander, Agent>
where
    Commander: SearchCommander,
    Agent: SearchAgent<Commander::Command>,
{
    type Error = SearchOperatorError<Agent::Error, Commander::Error>;

    fn tick(&self) -> Result<(), Self::Error> {
        self.agent
            .update()
            .map_err(|err| SearchOperatorError::Agent(err))
    }

    fn run(&self) -> Result<(), Result<IncompletedError, Self::Error>> {
        use SearchCommanderError::*;

        if let Some(is_full) = self.agent.is_full() {
            if is_full {
                return Err(Ok(IncompletedError));
            }
        } else {
            return Err(Ok(IncompletedError));
        }

        match self.commander.next_command() {
            Ok(command) => match self.agent.set_command(&command) {
                Ok(_) => Err(Ok(IncompletedError)),
                Err(err) => Err(Err(SearchOperatorError::Agent(err))),
            },
            Err(err) => match err {
                SearchFinish => {
                    if let Some(is_empty) = self.agent.is_empty() {
                        if is_empty {
                            return Ok(());
                        }
                    }
                    Err(Ok(IncompletedError))
                }
                Waiting => Err(Ok(IncompletedError)),
                Other(err) => Err(Err(SearchOperatorError::Commander(err))),
            },
        }
    }
}
