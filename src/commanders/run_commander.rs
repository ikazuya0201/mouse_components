use core::cmp::Reverse;
use core::fmt::Debug;

use generic_array::ArrayLength;
use heapless::Vec;
use num::{Bounded, Saturating};
use typenum::Unsigned;

use super::{
    compute_shortest_path, BoundedNode, BoundedPathNode, GoalSizeUpperBound, Graph, RouteNode,
};
use crate::operators::InitialCommander;

/// An implementation of [InitialCommander](crate::operators::InitialCommander).
pub struct RunCommander<Node, Maze> {
    start: Node,
    goals: Vec<Node, GoalSizeUpperBound>,
    maze: Maze,
}

impl<Node, Maze> RunCommander<Node, Maze>
where
    Node: Clone,
{
    pub fn new(start: Node, goals: &[Node], maze: Maze) -> Self {
        Self {
            start,
            goals: goals.into_iter().cloned().collect(),
            maze,
        }
    }
}

/// An implementation of [InitialCommander](crate::operators::InitialCommander).
///
/// This generates commands for return to start.
pub struct ReturnCommander<Node, Maze>(RunCommander<Node, Maze>);

impl<Node, Maze> ReturnCommander<Node, Maze>
where
    Node: Clone,
{
    pub fn new(start: Node, goals: &[Node], maze: Maze) -> Self {
        Self(RunCommander::new(start, goals, maze))
    }
}

impl<Node, Maze> InitialCommander for ReturnCommander<Node, Maze>
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

    fn initial_commands(&self) -> Result<Self::Commands, Self::Error> {
        self.0.initial_commands()
    }
}

impl<Node, Maze> core::ops::Deref for ReturnCommander<Node, Maze> {
    type Target = RunCommander<Node, Maze>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

/// Error on [RunCommander](RunCommander).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RunCommanderError {
    UnreachableError,
    ConversionError,
}

//TODO: write test
impl<Node, Maze> InitialCommander for RunCommander<Node, Maze>
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

    fn initial_commands(&self) -> Result<Self::Commands, Self::Error> {
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
