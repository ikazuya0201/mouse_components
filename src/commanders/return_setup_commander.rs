use heapless::ArrayLength;
use num::{Bounded, Saturating};
use spin::Mutex;
use typenum::Unsigned;

use super::{compute_shortest_path, BoundedNode, CostNode, Graph};
use crate::operators::{TrackingCommander, TrackingCommanderError};

/// An implementation of [TrackingCommander](crate::operators::TrackingCommander).
///
/// This produces a setup command for returning to start position.
pub struct ReturnSetupCommander<Node, Maze>
where
    Node: RotationNode,
{
    current: Node,
    maze: Maze,
    kind: Mutex<Option<Node::Kind>>,
}

impl<Node, Maze> ReturnSetupCommander<Node, Maze>
where
    Node: BoundedNode + Clone + Into<usize> + PartialEq + RotationNode,
    Node::UpperBound: Unsigned
        + ArrayLength<Maze::Cost>
        + ArrayLength<Option<Node>>
        + ArrayLength<CostNode<Maze::Cost, Node>>
        + ArrayLength<Option<usize>>,
    Maze: Graph<Node>,
    Maze::Cost: Bounded + Saturating + Copy + Ord,
{
    pub fn new(current: Node, start: Node, maze: Maze) -> Self {
        for (node, kind) in current.rotation_nodes() {
            if let Some(_) = compute_shortest_path(&node, &[start.clone()], &maze) {
                return Self {
                    current: node,
                    maze,
                    kind: Mutex::new(Some(kind)),
                };
            }
        }
        unreachable!()
    }
}

impl<Node, Maze> ReturnSetupCommander<Node, Maze>
where
    Node: RotationNode,
{
    pub fn release(self) -> (Node, Maze) {
        let Self { current, maze, .. } = self;
        (current, maze)
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

impl<Node, Maze> TrackingCommander for ReturnSetupCommander<Node, Maze>
where
    Node: BoundedNode + Clone + Into<usize> + PartialEq + RotationNode,
    Node::UpperBound: Unsigned
        + ArrayLength<Maze::Cost>
        + ArrayLength<Option<Node>>
        + ArrayLength<CostNode<Maze::Cost, Node>>
        + ArrayLength<Option<usize>>,
    Maze: Graph<Node>,
    Maze::Cost: Bounded + Saturating + Copy + Ord,
{
    type Error = ReturnSetupCommanderError;
    type Command = Node::Kind;

    fn next_command(&self) -> Result<Self::Command, TrackingCommanderError<Self::Error>> {
        if let Some(kind) = self.kind.lock().take() {
            Ok(kind)
        } else {
            Err(TrackingCommanderError::TrackingFinish)
        }
    }
}
