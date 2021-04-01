//! Implementations of commanders which is required by operators in [operators](crate::operators).

mod return_setup_commander;
mod run_commander;
mod search_commander;

use generic_array::{ArrayLength, GenericArray};
use heapless::{binary_heap::Min, consts::*, BinaryHeap, Vec};
use num::{Bounded, Saturating};
use typenum::Unsigned;

use crate::utils::{forced_vec::ForcedVec, itertools::repeat_n};
pub use return_setup_commander::{ReturnSetupCommander, ReturnSetupCommanderError, RotationNode};
pub use run_commander::{RunCommander, RunCommanderError};
pub use search_commander::{NextNode, NodeChecker, SearchCommander, UncheckedNodeFinder};

/// A trait that behaves like a directed graph which has weighted edges.
pub trait Graph<Node> {
    type Cost;
    type Edges: IntoIterator<Item = (Node, Self::Cost)>;

    fn successors(&self, node: &Node) -> Self::Edges;
    fn predecessors(&self, node: &Node) -> Self::Edges;
}

/// A trait that has a type level upper bound for the number of the given node type.
pub trait BoundedNode {
    type UpperBound;
}

/// A trait that computes a route value between itself and another node.
pub trait RouteNode {
    type Error;
    type Route;

    fn route(&self, to: &Self) -> Result<Self::Route, Self::Error>;
}

pub type GoalSizeUpperBound = U8;

#[derive(Debug)]
pub struct CostNode<Cost, Node>(Cost, Node);

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

pub type PathUpperBound = U1024; // Fixed upper bound of path length to 32x32.

fn compute_shortest_path<Node, Maze>(
    start: &Node,
    goals: &[Node],
    maze: &Maze,
) -> Option<Vec<Node, PathUpperBound>>
where
    Node: BoundedNode + Clone + Into<usize> + PartialEq,
    Node::UpperBound: Unsigned
        + ArrayLength<Maze::Cost>
        + ArrayLength<CostNode<Maze::Cost, Node>>
        + ArrayLength<Option<Node>>,
    Maze: Graph<Node>,
    Maze::Cost: Bounded + Saturating + Copy + Ord,
{
    let mut dists = repeat_n(
        Maze::Cost::max_value(),
        <Node::UpperBound as Unsigned>::USIZE,
    )
    .collect::<GenericArray<_, Node::UpperBound>>();
    dists[start.clone().into()] = Maze::Cost::min_value();

    let mut prev = repeat_n(None, <Node::UpperBound as Unsigned>::USIZE)
        .collect::<GenericArray<Option<Node>, Node::UpperBound>>();

    let mut heap = BinaryHeap::<CostNode<Maze::Cost, Node>, Node::UpperBound, Min>::new();
    heap.push(CostNode(Maze::Cost::min_value(), start.clone()))
        .unwrap_or_else(|_| {
            unreachable!("The length of binary heap should never exceed the upper bound")
        });

    let construct_path = |goal: Node, prev: GenericArray<Option<Node>, Node::UpperBound>| {
        let mut current = prev[goal.clone().into()].clone()?;
        let mut path = ForcedVec::new();
        path.push(goal);
        path.push(current.clone());
        while let Some(next) = prev[current.clone().into()].as_ref() {
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
        for goal in goals.iter() {
            if &node == goal {
                return construct_path(goal.clone(), prev);
            }
        }
        for (next, edge_cost) in maze.successors(&node) {
            let next_cost = cost.saturating_add(edge_cost);
            if next_cost < dists[next.clone().into()] {
                dists[next.clone().into()] = next_cost;
                heap.push(CostNode(next_cost, next.clone()))
                    .unwrap_or_else(|_| {
                        unreachable!(
                            "The length of binary heap should never exceed the upper bound"
                        )
                    });
                prev[next.into()] = Some(node.clone());
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use std::vec::Vec;

    use heapless::consts::*;

    use super::*;

    #[test]
    fn test_compute_shortest_path() {
        struct Node(usize);
        #[derive(Clone, Copy, PartialEq, Debug)]
        struct RunNode(usize);
        struct SearchNode(usize);

        impl From<usize> for RunNode {
            fn from(value: usize) -> Self {
                RunNode(value)
            }
        }

        impl From<RunNode> for usize {
            fn from(value: RunNode) -> Self {
                value.0
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
            usize: From<T>,
            T: Clone + From<usize>,
        {
            type Cost = usize;
            type Edges = Vec<(T, usize)>;

            fn successors(&self, node: &T) -> Self::Edges {
                let node = usize::from(node.clone());
                let mut result = Vec::new();
                for i in 0..self.n {
                    if let Some(cost) = self.mat[node][i] {
                        result.push((T::from(i), cost));
                    }
                }
                result
            }

            fn predecessors(&self, node: &T) -> Self::Edges {
                let node = usize::from(node.clone());
                let mut result = Vec::new();
                for i in 0..self.n {
                    if let Some(cost) = self.mat[i][node] {
                        result.push((T::from(i), cost));
                    }
                }
                result
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
        let goals = vec![RunNode(8usize)];
        let n = 9;

        let graph = IGraph::new(n, &edges);

        impl BoundedNode for RunNode {
            type UpperBound = U10;
        }

        let solver = SearchCommander::<Node, RunNode, SearchNode, _, _>::new(
            start.clone(),
            &goals,
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
