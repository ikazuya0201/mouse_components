use core::fmt::Debug;
use core::sync::atomic::{AtomicUsize, Ordering};

use generic_array::ArrayLength;
use heapless::Vec;
use num::{Bounded, Saturating};
use typenum::Unsigned;

use super::{compute_shortest_path, BoundedNode, CostNode, Graph, PathUpperBound, RouteNode};
use crate::operators::{TrackingCommander, TrackingCommanderError};

/// An implementation of [TrackingCommander](crate::operators::TrackingCommander).
pub struct RunCommander<Node, Maze>
where
    Node: RouteNode,
{
    current: Node,
    maze: Maze,
    path: Vec<Node, PathUpperBound>,
    iter: AtomicUsize,
}

impl<Node, Maze> RunCommander<Node, Maze>
where
    Node::UpperBound: ArrayLength<Option<Node>>
        + ArrayLength<Option<usize>>
        + ArrayLength<Maze::Cost>
        + ArrayLength<CostNode<Maze::Cost, Node>>
        + ArrayLength<(Node, Node::Route)>
        + Unsigned,
    Node: PartialEq + Copy + Debug + Into<usize> + RouteNode + BoundedNode,
    Node: From<Node>,
    Maze::Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<Node>,
{
    pub fn new(start: Node, goals: &[Node], maze: Maze) -> Self {
        let path = compute_shortest_path(&start, &goals, &maze).unwrap_or_else(|| unimplemented!());

        Self {
            current: path.last().unwrap_or_else(|| unimplemented!()).clone(),
            maze,
            path,
            iter: AtomicUsize::new(0),
        }
    }

    pub fn release(self) -> (Node, Maze) {
        let Self { current, maze, .. } = self;
        (current, maze)
    }
}

/// Error on [RunCommander](RunCommander).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RunCommanderError {
    UnreachableError,
    ConversionError,
}

//TODO: write test
impl<Node, Maze> TrackingCommander for RunCommander<Node, Maze>
where
    Node: RouteNode + Clone,
{
    type Error = RunCommanderError;
    type Command = (Node, Node::Route);

    fn next_command(&self) -> Result<Self::Command, TrackingCommanderError<Self::Error>> {
        let iter = self.iter.load(Ordering::Acquire);
        if iter + 1 >= self.path.len() {
            return Err(TrackingCommanderError::TrackingFinish);
        }
        self.iter.fetch_add(1, Ordering::AcqRel);
        Ok((
            self.path[iter].clone(),
            self.path[iter]
                .route(&self.path[iter + 1])
                .map_err(|_| TrackingCommanderError::Other(RunCommanderError::ConversionError))?,
        ))
    }
}
