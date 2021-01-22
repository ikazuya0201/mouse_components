use core::sync::atomic::{AtomicBool, Ordering};

use crate::administrator::{IncompletedError, Operator};

#[derive(Debug)]
pub enum UpdateError<T> {
    EmptyTrajectory,
    Other(T),
}

pub trait SearchAgent<Command> {
    type Error;

    fn update(&self) -> Result<(), UpdateError<Self::Error>>;
    fn set_command(&self, command: &Command) -> Result<(), Self::Error>;
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
    trajectory_is_empty: AtomicBool,
}

impl<Commander, Agent> SearchOperator<Commander, Agent> {
    pub fn new(commander: Commander, agent: Agent) -> Self {
        Self {
            commander,
            agent,
            trajectory_is_empty: AtomicBool::new(false),
        }
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
        match self.agent.update() {
            Ok(_) => {
                self.trajectory_is_empty.store(false, Ordering::Release);
                Ok(())
            }
            Err(err) => match err {
                UpdateError::EmptyTrajectory => {
                    self.trajectory_is_empty.store(true, Ordering::Release);
                    Ok(())
                }
                UpdateError::Other(err) => Err(SearchOperatorError::Agent(err)),
            },
        }
    }

    fn run(&self) -> Result<(), Result<IncompletedError, Self::Error>> {
        use SearchCommanderError::*;

        match self.commander.next_command() {
            Ok(command) => match self.agent.set_command(&command) {
                Ok(_) => Err(Ok(IncompletedError)),
                Err(err) => Err(Err(SearchOperatorError::Agent(err))),
            },
            Err(err) => match err {
                SearchFinish => {
                    if self.trajectory_is_empty.load(Ordering::Acquire) {
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
