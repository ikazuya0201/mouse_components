use crate::administrator::{Operator, OperatorError};
use crate::{Construct, Deconstruct, Merge};

/// A trait that processes command and moves the machine.
///
/// Physical quantities should be maanged only in this trait.
pub trait TrackingAgent<Command> {
    type Error;

    fn update(&self) -> Result<(), Self::Error>;
    fn set_command(&self, command: &Command) -> Result<(), Self::Error>;
    fn is_full(&self) -> Option<bool>;
    fn is_empty(&self) -> Option<bool>;
    /// Stops all actuators in agent.
    fn stop(&self);
}

/// Error on [TrackingCommander](TrackingCommander)
///
/// - TrackingFinish indicates the tracking process has completed.
/// - Waiting indicates the calculation process of next command is being blocked now.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TrackingCommanderError<T> {
    TrackingFinish,
    Waiting,
    Other(T),
}

/// A trait that calculates and instructs commands for agent.
pub trait TrackingCommander {
    type Error;
    type Command;

    fn next_command(&self) -> Result<Self::Command, TrackingCommanderError<Self::Error>>;
}

/// An implementation of [Operator](crate::administrator::Operator) required by
/// [Administrator](crate::administrator::Administrator).
///
/// This processes a search of a maze.
pub struct TrackingOperator<Commander, Agent> {
    commander: Commander,
    agent: Agent,
}

impl<Commander, Agent> TrackingOperator<Commander, Agent>
where
    Commander: TrackingCommander,
    Agent: TrackingAgent<Commander::Command>,
{
    pub fn new(commander: Commander, agent: Agent) -> Self {
        let command = commander.next_command().unwrap_or_else(|_| unreachable!());
        agent
            .set_command(&command)
            .unwrap_or_else(|_| unreachable!());
        Self { commander, agent }
    }
}

impl<Commander, Agent> TrackingOperator<Commander, Agent> {
    pub fn release(self) -> (Commander, Agent) {
        let Self { commander, agent } = self;
        (commander, agent)
    }
}

impl<Commander, Agent, Config, State, Resource> Construct<Config, State, Resource>
    for TrackingOperator<Commander, Agent>
where
    Commander: Construct<Config, State, Resource> + TrackingCommander,
    Agent: Construct<Config, State, Resource> + TrackingAgent<Commander::Command>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let commander = Commander::construct(config, state, resource);
        let agent = Agent::construct(config, state, resource);
        Self::new(commander, agent)
    }
}

impl<Commander, Agent, State, Resource> Deconstruct<State, Resource>
    for TrackingOperator<Commander, Agent>
where
    Commander: Deconstruct<State, Resource> + TrackingCommander,
    Agent: Deconstruct<State, Resource> + TrackingAgent<Commander::Command>,
    State: Merge,
    Resource: Merge,
{
    fn deconstruct(self) -> (State, Resource) {
        let (commander, agent) = self.release();
        let (commander_state, commander_resource) = commander.deconstruct();
        let (agent_state, agent_resource) = agent.deconstruct();
        (
            commander_state.merge(agent_state),
            commander_resource.merge(agent_resource),
        )
    }
}

#[derive(Debug)]
pub enum TrackingOperatorError<T, U> {
    Agent(T),
    Commander(U),
}

impl<Commander, Agent> Operator for TrackingOperator<Commander, Agent>
where
    Commander: TrackingCommander,
    Agent: TrackingAgent<Commander::Command>,
{
    type Error = TrackingOperatorError<Agent::Error, Commander::Error>;

    fn tick(&self) -> Result<(), Self::Error> {
        self.agent.update().map_err(TrackingOperatorError::Agent)
    }

    fn run(&self) -> Result<(), OperatorError<Self::Error>> {
        use TrackingCommanderError::*;

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
                Err(err) => Err(OperatorError::Other(TrackingOperatorError::Agent(err))),
            },
            Err(err) => match err {
                TrackingFinish => {
                    if let Some(is_empty) = self.agent.is_empty() {
                        if is_empty {
                            return Ok(());
                        }
                    }
                    Err(OperatorError::Incompleted)
                }
                Waiting => Err(OperatorError::Incompleted),
                Other(err) => Err(OperatorError::Other(TrackingOperatorError::Commander(err))),
            },
        }
    }

    fn stop(&self) {
        self.agent.stop();
    }
}
