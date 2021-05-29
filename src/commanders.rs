//! Implementations of commanders which is required by operators in [operators](crate::operators).

mod run;
mod search;
mod setup;

use heapless::Vec;
use serde::{Deserialize, Serialize};

use crate::MAZE_WIDTH_UPPER_BOUND;
pub use run::{
    ReturnCommander, ReturnCommanderConfig, RunCommand, RunCommander, RunCommanderConfig,
    RunCommanderError,
};
pub use search::{
    NextNode, NodeChecker, SearchCommander, SearchCommanderConfig, UncheckedNodeFinder,
};
pub use setup::{
    ReturnSetupCommander, ReturnSetupCommanderConfig, RotationNode, RunSetupCommander,
    RunSetupCommanderConfig, SetupCommander, SetupCommanderError,
};

/// A trait that behaves like a directed graph which has weighted edges.
pub trait Graph<Node> {
    type Cost;
    type Edges: IntoIterator<Item = (Node, Self::Cost)>;

    fn successors(&self, node: &Node) -> Self::Edges;
    fn predecessors(&self, node: &Node) -> Self::Edges;
}

/// A trait that represents a geometic graph.
pub trait GeometricGraph<Node>: Graph<Node> {
    fn distance(&self, lhs: &Node, rhs: &Node) -> Self::Cost;
}

/// A trait that computes a route value between itself and another node.
pub trait RouteNode {
    type Error;
    type Route;

    fn route(&self, to: &Self) -> Result<Self::Route, Self::Error>;
}

/// Solver for single source shortest path.
pub trait SsspSolver<Node, Maze> {
    type Error;
    type Path;

    fn solve(&self, start: &Node, goals: &[Node], maze: &Maze) -> Result<Self::Path, Self::Error>;
}

const GOAL_SIZE_UPPER_BOUND: usize = 8;

pub(crate) type GoalVec<T> = Vec<T, GOAL_SIZE_UPPER_BOUND>;

#[derive(Debug)]
struct CostNode<Cost, Node>(Cost, Node);

impl<Cost: PartialEq, Node> PartialEq for CostNode<Cost, Node> {
    fn eq(&self, other: &Self) -> bool {
        self.0.eq(&other.0)
    }
}

impl<Cost: Eq, Node> Eq for CostNode<Cost, Node> {}

impl<Cost: PartialOrd, Node> PartialOrd for CostNode<Cost, Node> {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        self.0.partial_cmp(&other.0)
    }
}

impl<Cost: Ord, Node> Ord for CostNode<Cost, Node> {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.0.cmp(&other.0)
    }
}

/// State for commanders.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct CommanderState<Node> {
    pub current_node: Node,
}

pub(crate) const PATH_UPPER_BOUND: usize = MAZE_WIDTH_UPPER_BOUND * MAZE_WIDTH_UPPER_BOUND;

pub(crate) const NODE_NUMBER_UPPER_BOUND: usize =
    16 * MAZE_WIDTH_UPPER_BOUND * MAZE_WIDTH_UPPER_BOUND;

pub(crate) const HEAP_SIZE: usize = 8 * MAZE_WIDTH_UPPER_BOUND * MAZE_WIDTH_UPPER_BOUND;

/// A trait that can be interpreted as index.
pub trait AsIndex {
    fn as_index(&self) -> usize;
}

/// A trait that can be converted into an unique ID.
pub trait AsId {
    type Id;

    fn as_id(&self) -> Self::Id;
}
