//! Implementations of commanders which is required by operators in [operators](crate::operators).

mod run_commander;
mod search_commander;
mod setup_commander;

use heapless::{binary_heap::Min, BinaryHeap, Vec};
use num_traits::{Bounded, PrimInt, Saturating, Unsigned};
use serde::{Deserialize, Serialize};

use crate::utils::forced_vec::ForcedVec;
use crate::MAZE_WIDTH_UPPER_BOUND;
pub use run_commander::{
    ReturnCommander, ReturnCommanderConfig, RunCommand, RunCommander, RunCommanderConfig,
    RunCommanderError,
};
pub use search_commander::{
    NextNode, NodeChecker, SearchCommander, SearchCommanderConfig, UncheckedNodeFinder,
};
pub use setup_commander::{
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

const HEAP_SIZE: usize = 3000;

/// A trait that can be interpreted as index.
pub trait AsIndex {
    fn as_index(&self) -> usize;
}

// NOTE: Distances between `goals` must be 0.
// The length of `goals` must be greater than 0.
fn compute_shortest_path<Node, Maze>(
    start: &Node,
    goals: &[Node],
    maze: &Maze,
) -> Option<Vec<Node, PATH_UPPER_BOUND>>
where
    Node: Clone + AsIndex + PartialEq,
    Maze: GeometricGraph<Node>,
    Maze::Cost: PrimInt + Unsigned,
{
    let mut dists = [Maze::Cost::max_value(); NODE_NUMBER_UPPER_BOUND];
    let start_cost = Maze::Cost::min_value() + maze.distance(&start, &goals[0]);
    dists[start.as_index()] = start_cost;

    let mut prev = core::iter::repeat(None)
        .take(NODE_NUMBER_UPPER_BOUND)
        .collect::<Vec<_, NODE_NUMBER_UPPER_BOUND>>();

    let mut heap = BinaryHeap::<CostNode<Maze::Cost, Node>, Min, HEAP_SIZE>::new();
    heap.push(CostNode(start_cost, start.clone()))
        .unwrap_or_else(|_| {
            unreachable!("The length of binary heap should never exceed the upper bound")
        });

    let construct_path = |goal: Node, prev: &[Option<Node>]| {
        let mut current = prev[goal.as_index()].clone()?;
        let mut path = ForcedVec::new();
        path.push(goal);
        path.push(current.clone());
        while let Some(next) = prev[current.as_index()].as_ref() {
            path.push(next.clone());
            current = next.clone();
        }
        let len = path.len();
        //reverse
        for i in 0..len / 2 {
            path.swap(i, len - i - 1);
        }
        Some(path.into())
    };

    while let Some(CostNode(cost, node)) = heap.pop() {
        if dists[node.as_index()] < cost {
            continue;
        }
        for goal in goals.iter() {
            if &node == goal {
                return construct_path(goal.clone(), &prev);
            }
        }
        for (next, edge_cost) in maze.successors(&node) {
            let next_cost = cost.saturating_add(edge_cost + maze.distance(&next, &goals[0]));
            let next_index = next.as_index();
            if next_cost < dists[next_index] {
                dists[next_index] = next_cost;
                heap.push(CostNode(next_cost, next)).unwrap_or_else(|_| {
                    unreachable!("The length of binary heap should never exceed the upper bound")
                });
                prev[next_index] = Some(node.clone());
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use std::vec::Vec;

    use super::*;

    #[test]
    fn test_compute_shortest_path() {
        struct Node(usize);
        #[derive(Clone, Copy, PartialEq, Debug)]
        struct RunNode(usize);
        struct SearchNode(usize);

        #[derive(Clone)]
        struct Position(usize);

        impl Into<GoalVec<RunNode>> for Position {
            fn into(self) -> GoalVec<RunNode> {
                core::array::IntoIter::new([RunNode(self.0)]).collect()
            }
        }

        impl From<usize> for RunNode {
            fn from(value: usize) -> Self {
                RunNode(value)
            }
        }

        impl AsIndex for RunNode {
            fn as_index(&self) -> usize {
                self.0
            }
        }

        impl From<RunNode> for Node {
            fn from(value: RunNode) -> Self {
                Node(value.0)
            }
        }

        struct IGraph {
            n: usize,
            mat: Vec<Vec<Option<usize>>>,
        }

        impl IGraph {
            fn new(n: usize, edges: &[(usize, usize, usize)]) -> Self {
                let mut mat = vec![vec![None; n]; n];
                for &(src, dst, cost) in edges {
                    mat[src][dst] = Some(cost);
                }
                Self { n, mat }
            }
        }

        impl<T> Graph<T> for IGraph
        where
            T: Clone + From<usize> + AsIndex,
        {
            type Cost = usize;
            type Edges = Vec<(T, usize)>;

            fn successors(&self, node: &T) -> Self::Edges {
                let node = node.as_index();
                let mut result = Vec::new();
                for i in 0..self.n {
                    if let Some(cost) = self.mat[node][i] {
                        result.push((T::from(i), cost));
                    }
                }
                result
            }

            fn predecessors(&self, node: &T) -> Self::Edges {
                let node = node.as_index();
                let mut result = Vec::new();
                for i in 0..self.n {
                    if let Some(cost) = self.mat[i][node] {
                        result.push((T::from(i), cost));
                    }
                }
                result
            }
        }

        impl<T> GeometricGraph<T> for IGraph
        where
            T: Clone + From<usize> + AsIndex,
        {
            fn distance(&self, _lhs: &T, _rhs: &T) -> Self::Cost {
                0
            }
        }

        let edges = [
            (0, 1, 2),
            (0, 2, 1),
            (1, 3, 1),
            (2, 3, 3),
            (3, 4, 2),
            (3, 5, 5),
            (3, 6, 4),
            (5, 6, 3),
            (5, 7, 7),
            (7, 8, 1),
        ];
        let start = RunNode(0);
        let goal = Position(8usize);
        let n = 9;

        let graph = IGraph::new(n, &edges);

        let solver = SearchCommander::<Node, RunNode, SearchNode, _, _, _>::new(
            start.clone(),
            goal,
            start.into(),
            (),
            (),
            graph,
        );

        let path = solver.compute_shortest_path();
        let expected = vec![0, 1, 3, 5, 7, 8]
            .into_iter()
            .map(|e| RunNode(e))
            .collect::<Vec<_>>();

        assert!(path.is_some());
        assert_eq!(path.unwrap(), expected.as_slice());
    }
}
