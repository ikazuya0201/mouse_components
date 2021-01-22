use crate::administrator::{IncompletedError, Operator};

pub trait SearchAgent<Command> {
    type Error;

    fn update(&self) -> Result<(), Self::Error>;
    fn set_command(&self, command: &Command) -> Result<(), Self::Error>;
    fn is_full(&self) -> bool;
    fn is_empty(&self) -> bool;
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SearchCommanderError<T> {
    SearchFinish,
    Waiting,
    Other(T),
}

pub trait SearchCommander {
    type Error;
    type Command;

    fn next_command(&self) -> Result<Self::Command, SearchCommanderError<Self::Error>>;
}

pub struct SearchOperator<Commander, Agent> {
    commander: Commander,
    agent: Agent,
}

impl<Commander, Agent> SearchOperator<Commander, Agent> {
    pub fn new(commander: Commander, agent: Agent) -> Self {
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

        if self.agent.is_full() {
            return Err(Ok(IncompletedError));
        }

        match self.commander.next_command() {
            Ok(command) => match self.agent.set_command(&command) {
                Ok(_) => Err(Ok(IncompletedError)),
                Err(err) => Err(Err(SearchOperatorError::Agent(err))),
            },
            Err(err) => match err {
                SearchFinish => {
                    if self.agent.is_empty() {
                        Ok(())
                    } else {
                        Err(Ok(IncompletedError))
                    }
                }
                Waiting => Err(Ok(IncompletedError)),
                Other(err) => Err(Err(SearchOperatorError::Commander(err))),
            },
        }
    }
}
