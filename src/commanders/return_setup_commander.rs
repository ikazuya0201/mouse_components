use core::cmp::Reverse;

use heapless::ArrayLength;
use num::{Bounded, Saturating};
use typenum::Unsigned;

use super::{compute_shortest_path, BoundedNode, BoundedPathNode, CommanderState, Graph};
use crate::operators::InitialCommander;

/// An implementation of [InitialCommander](crate::operators::InitialCommander).
///
/// This produces a setup command for returning to start position.
pub struct ReturnSetupCommander<Node, Maze> {
    current: Node,
    start: Node,
    maze: Maze,
}

impl<Node, Maze> ReturnSetupCommander<Node, Maze> {
    pub fn new(current: Node, start: Node, maze: Maze) -> Self {
        Self {
            current,
            start,
            maze,
        }
    }
}

/// A config for initializing [ReturnSetupCommander](ReturnSetupCommander).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReturnSetupCommanderConfig<Node> {
    pub start: Node,
}

impl<'a, Resource, Config, State, Node, Maze> From<(Resource, &'a Config, &'a State)>
    for ReturnSetupCommander<Node, Maze>
where
    Maze: From<(Resource, &'a Config, &'a State)>,
    &'a Config: Into<ReturnSetupCommanderConfig<Node>>,
    &'a State: Into<CommanderState<Node>>,
{
    fn from((resource, config, state): (Resource, &'a Config, &'a State)) -> Self {
        let maze = Maze::from((resource, config, state));
        let config = config.into();
        let state = state.into();
        Self::new(state.current_node, config.start, maze)
    }
}

/// Error on [ReturnSetupCommander](ReturnSetupCommander).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ReturnSetupCommanderError;

/// A trait that enumerates all nodes which is reachable from the given node, and also enumerates
/// kinds of rotation.
pub trait RotationNode: Sized {
    type Kind;
    type Nodes: IntoIterator<Item = (Self, Self::Kind)>;

    fn rotation_nodes(&self) -> Self::Nodes;
}

impl<Node, Maze> InitialCommander for ReturnSetupCommander<Node, Maze>
where
    Node: BoundedPathNode + BoundedNode + Clone + Into<usize> + PartialEq + RotationNode,
    Node::PathUpperBound: ArrayLength<Node>,
    Node::UpperBound: Unsigned
        + ArrayLength<Maze::Cost>
        + ArrayLength<Option<Node>>
        + ArrayLength<(Node, Reverse<Maze::Cost>)>
        + ArrayLength<Option<usize>>,
    Maze: Graph<Node>,
    Maze::Cost: Bounded + Saturating + Copy + Ord,
{
    type Error = ReturnSetupCommanderError;
    type Command = Node::Kind;
    type Commands = Option<Self::Command>;

    fn initial_commands(&self) -> Result<Self::Commands, Self::Error> {
        for (node, kind) in self.current.rotation_nodes() {
            if let Some(_) = compute_shortest_path(&node, &[self.start.clone()], &self.maze) {
                return Ok(Some(kind));
            }
        }
        Err(ReturnSetupCommanderError)
    }
}
