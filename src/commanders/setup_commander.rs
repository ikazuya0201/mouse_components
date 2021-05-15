use heapless::Vec;
use num::{Bounded, Saturating};
use serde::{Deserialize, Serialize};
use spin::Mutex;

use super::{compute_shortest_path, AsIndex, CommanderState, Graph, GOAL_SIZE_UPPER_BOUND};
use crate::operators::{TrackingCommander, TrackingCommanderError};
use crate::{Construct, Deconstruct, Merge};

/// An implementation of [TrackingCommander](crate::operators::TrackingCommander).
///
/// This produces a setup command.
pub struct SetupCommander<Node, Maze>
where
    Node: RotationNode,
{
    current: Node,
    maze: Maze,
    command: Mutex<Option<(Node, Node::Kind)>>,
}

impl<Node, Maze> SetupCommander<Node, Maze>
where
    Node: Clone + AsIndex + PartialEq + RotationNode,
    Maze: Graph<Node>,
    Maze::Cost: Bounded + Saturating + Copy + Ord,
{
    pub fn new(current: Node, goals: &[Node], maze: Maze) -> Self {
        for (node, kind) in current.rotation_nodes() {
            if let Some(_) = compute_shortest_path(&node, goals, &maze) {
                return Self {
                    current: node,
                    maze,
                    command: Mutex::new(Some((current, kind))),
                };
            }
        }
        unreachable!()
    }
}

impl<Node, Maze> SetupCommander<Node, Maze>
where
    Node: RotationNode,
{
    pub fn release(self) -> (Node, Maze) {
        let Self { current, maze, .. } = self;
        (current, maze)
    }
}

impl<Node, Maze, State, Resource> Deconstruct<State, Resource> for SetupCommander<Node, Maze>
where
    Node: RotationNode,
    Maze: Deconstruct<State, Resource>,
    State: From<CommanderState<Node>> + Merge,
{
    fn deconstruct(self) -> (State, Resource) {
        let (current_node, maze) = self.release();
        let (state, resource) = maze.deconstruct();
        (
            state.merge(CommanderState { current_node }.into()),
            resource,
        )
    }
}

/// Config for [ReturnSetupCommander].
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct ReturnSetupCommanderConfig<Node> {
    pub return_goal: Node,
}

/// A commander for setting up return.
pub struct ReturnSetupCommander<Node, Maze>(SetupCommander<Node, Maze>)
where
    Node: RotationNode;

impl<Node, Maze, Config, State, Resource> Construct<Config, State, Resource>
    for ReturnSetupCommander<Node, Maze>
where
    Node: Clone + AsIndex + PartialEq + RotationNode,
    Maze: Construct<Config, State, Resource> + Graph<Node>,
    Maze::Cost: Ord + Bounded + Saturating + Copy,
    Config: AsRef<ReturnSetupCommanderConfig<Node>>,
    State: AsRef<CommanderState<Node>>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let maze = Maze::construct(config, state, resource);
        let config = config.as_ref();
        let state = state.as_ref();
        Self(SetupCommander::new(
            state.current_node.clone(),
            &[config.return_goal.clone()],
            maze,
        ))
    }
}

/// Config for [ReturnSetupCommander].
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct RunSetupCommanderConfig<Node> {
    pub goals: Vec<Node, GOAL_SIZE_UPPER_BOUND>,
}

/// A commander for setting up fast run.
pub struct RunSetupCommander<Node, Maze>(SetupCommander<Node, Maze>)
where
    Node: RotationNode;

impl<Node, Maze, Config, State, Resource> Construct<Config, State, Resource>
    for RunSetupCommander<Node, Maze>
where
    Node: Clone + AsIndex + PartialEq + RotationNode,
    Maze: Construct<Config, State, Resource> + Graph<Node>,
    Maze::Cost: Ord + Bounded + Saturating + Copy,
    Config: AsRef<RunSetupCommanderConfig<Node>>,
    State: AsRef<CommanderState<Node>>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let maze = Maze::construct(config, state, resource);
        let config = config.as_ref();
        let state = state.as_ref();
        Self(SetupCommander::new(
            state.current_node.clone(),
            &config.goals,
            maze,
        ))
    }
}

/// Error on [ReturnSetupCommander](ReturnSetupCommander).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SetupCommanderError;

/// A trait that enumerates all nodes which is reachable from the given node, and also enumerates
/// kinds of rotation.
pub trait RotationNode: Sized {
    type Kind;
    type Nodes: IntoIterator<Item = (Self, Self::Kind)>;

    fn rotation_nodes(&self) -> Self::Nodes;
}

//TODO: Write test.
impl<Node, Maze> TrackingCommander for SetupCommander<Node, Maze>
where
    Node: Clone + AsIndex + PartialEq + RotationNode,
    Maze: Graph<Node>,
    Maze::Cost: Bounded + Saturating + Copy + Ord,
{
    type Error = SetupCommanderError;
    type Command = (Node, Node::Kind);

    fn next_command(&self) -> Result<Self::Command, TrackingCommanderError<Self::Error>> {
        if let Some(command) = self.command.lock().take() {
            Ok(command)
        } else {
            Err(TrackingCommanderError::TrackingFinish)
        }
    }
}

macro_rules! impl_all_for_setup_commander {
    ($name: ident) => {
        impl<Node, Maze, State, Resource> Deconstruct<State, Resource> for $name<Node, Maze>
        where
            Node: RotationNode,
            Maze: Deconstruct<State, Resource>,
            State: From<CommanderState<Node>> + Merge,
        {
            fn deconstruct(self) -> (State, Resource) {
                self.0.deconstruct()
            }
        }

        impl<Node, Maze> TrackingCommander for $name<Node, Maze>
        where
            Node: Clone + AsIndex + PartialEq + RotationNode,
            Maze: Graph<Node>,
            Maze::Cost: Bounded + Saturating + Copy + Ord,
        {
            type Error = SetupCommanderError;
            type Command = (Node, Node::Kind);

            fn next_command(&self) -> Result<Self::Command, TrackingCommanderError<Self::Error>> {
                self.0.next_command()
            }
        }
    };
}

impl_all_for_setup_commander!(ReturnSetupCommander);
impl_all_for_setup_commander!(RunSetupCommander);
