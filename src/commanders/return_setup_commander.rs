use core::cmp::Reverse;

use heapless::ArrayLength;
use num::{Bounded, Saturating};
use typenum::Unsigned;

use super::{compute_shortest_path, BoundedNode, BoundedPathNode, Graph};
use crate::operators::InitialCommander;

/// An implementation of [ReturnSetupCommander](crate::operators::ReturnSetupCommander).
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
