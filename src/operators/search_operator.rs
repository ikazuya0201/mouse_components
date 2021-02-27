use crate::administrator::{Operator, OperatorError};

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

impl<'a, CResource, AResource, State, Config, Commander, Agent>
    From<((CResource, AResource), &'a State, &'a Config)> for SearchOperator<Commander, Agent>
where
    Commander: From<(CResource, &'a State, &'a Config)> + SearchCommander,
    Agent: From<(AResource, &'a State, &'a Config)> + SearchAgent<Commander::Command>,
{
    fn from(
        ((commander, agent), state, config): ((CResource, AResource), &'a State, &'a Config),
    ) -> Self {
        Self::new(
            Commander::from((commander, state, config)),
            Agent::from((agent, state, config)),
        )
    }
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

    fn run(&self) -> Result<(), OperatorError<Self::Error>> {
        use SearchCommanderError::*;

        if let Some(is_full) = self.agent.is_full() {
            if is_full {
                return Err(OperatorError::Incompleted);
            }
        } else {
            return Err(OperatorError::Incompleted);
        }

        match self.commander.next_command() {
            Ok(command) => match self.agent.set_command(&command) {
                Ok(_) => Err(OperatorError::Incompleted),
                Err(err) => Err(OperatorError::Other(SearchOperatorError::Agent(err))),
            },
            Err(err) => match err {
                SearchFinish => {
                    if let Some(is_empty) = self.agent.is_empty() {
                        if is_empty {
                            return Ok(());
                        }
                    }
                    Err(OperatorError::Incompleted)
                }
                Waiting => Err(OperatorError::Incompleted),
                Other(err) => Err(OperatorError::Other(SearchOperatorError::Commander(err))),
            },
        }
    }
}
