use core::cmp::Reverse;
use core::fmt::Debug;

use generic_array::ArrayLength;
use heapless::Vec;
use num::{Bounded, Saturating};
use typenum::Unsigned;

use super::{
    compute_shortest_path, BoundedNode, BoundedPathNode, GoalSizeUpperBound, Graph, RouteNode,
};
use crate::operators::RunCommander as IRunCommander;

pub struct RunCommander<Node, Maze> {
    start: Node,
    goals: Vec<Node, GoalSizeUpperBound>,
    maze: Maze,
}

impl<Node, Maze> RunCommander<Node, Maze> {
    pub fn new<Nodes: IntoIterator<Item = Node>>(start: Node, goals: Nodes, maze: Maze) -> Self {
        Self {
            start,
            goals: goals.into_iter().collect(),
            maze,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RunCommanderError {
    UnreachableError,
    ConversionError,
}

///NOTO: multiple implementations of Converter<_> can exist for different Target,
///but we assume there is only an implementation.
//TODO: write test
impl<Node, Maze> IRunCommander for RunCommander<Node, Maze>
where
    Node::UpperBound: ArrayLength<Option<Node>>
        + ArrayLength<Option<usize>>
        + ArrayLength<Maze::Cost>
        + ArrayLength<Reverse<Maze::Cost>>
        + ArrayLength<(Node, Reverse<Maze::Cost>)>
        + ArrayLength<(Node, Node::Route)>
        + Unsigned,
    Node::PathUpperBound: ArrayLength<Node>,
    Node: PartialEq + Copy + Debug + Into<usize> + RouteNode + BoundedNode + BoundedPathNode,
    Node: From<Node>,
    Maze::Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<Node>,
{
    type Error = RunCommanderError;
    type Command = (Node, Node::Route);
    type Commands = Vec<Self::Command, Node::UpperBound>;

    fn compute_commands(&self) -> Result<Self::Commands, Self::Error> {
        let path = compute_shortest_path(&self.start, &self.goals, &self.maze)
            .ok_or(RunCommanderError::UnreachableError)?;

        let mut commands = Vec::new();
        if path.len() == 0 {
            return Ok(commands);
        }
        for i in 0..path.len() - 1 {
            commands
                .push((
                    Node::from(path[i]),
                    path[i]
                        .route(&path[i + 1])
                        .map_err(|_| RunCommanderError::ConversionError)?,
                ))
                .unwrap_or_else(|_| unreachable!());
        }
        Ok(commands)
    }
}
