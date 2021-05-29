use heapless::{binary_heap::Min, BinaryHeap, Vec};
use num_traits::{Bounded, PrimInt, Saturating, Unsigned};

use crate::commanders::{
    AsId, AsIndex, GeometricGraph, SsspSolver, HEAP_SIZE, NODE_NUMBER_UPPER_BOUND, PATH_UPPER_BOUND,
};
use crate::utils::forced_vec::ForcedVec;
use crate::{impl_deconstruct_with_default, Construct};

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

/// Error on [DijkstraSolver].
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct UnfeasibleError;

/// An implementation of [SsspSolver](crate::commanders::SsspSolver).
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct DijkstraSolver;

impl<Node, Maze> SsspSolver<Node, Maze> for DijkstraSolver
where
    Node: Clone + AsIndex + PartialEq + AsId + From<Node::Id>,
    Maze: GeometricGraph<Node>,
    Maze::Cost: PrimInt + Unsigned,
    Node::Id: Clone,
{
    type Error = UnfeasibleError;
    type Path = Vec<Node, PATH_UPPER_BOUND>;

    fn solve(&self, start: &Node, goals: &[Node], maze: &Maze) -> Result<Self::Path, Self::Error> {
        let mut dists = [Maze::Cost::max_value(); NODE_NUMBER_UPPER_BOUND];
        let start_cost = Maze::Cost::min_value() + maze.distance(&start, &goals[0]);
        dists[start.as_index()] = start_cost;

        let mut prev = core::iter::repeat(None)
            .take(NODE_NUMBER_UPPER_BOUND)
            .collect::<Vec<_, NODE_NUMBER_UPPER_BOUND>>();

        let mut heap = BinaryHeap::<CostNode<Maze::Cost, Node::Id>, Min, HEAP_SIZE>::new();
        heap.push(CostNode(start_cost, start.as_id()))
            .unwrap_or_else(|_| {
                unreachable!("The length of binary heap should never exceed the upper bound")
            });

        let construct_path = |goal: Node, prev: &[Option<Node::Id>]| {
            let mut current = Node::from(prev[goal.as_index()].clone().ok_or(UnfeasibleError)?);
            let mut path = ForcedVec::new();
            path.push(goal);
            path.push(current.clone());
            while let Some(next) = prev[current.as_index()].as_ref() {
                let next = Node::from(next.clone());
                path.push(next.clone());
                current = next;
            }
            let len = path.len();
            //reverse
            for i in 0..len / 2 {
                path.swap(i, len - i - 1);
            }
            Ok(path.into())
        };

        while let Some(CostNode(cost, node)) = heap.pop() {
            let node = Node::from(node);
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
                    heap.push(CostNode(next_cost, next.as_id()))
                        .unwrap_or_else(|_| {
                            unreachable!(
                                "The length of binary heap should never exceed the upper bound"
                            )
                        });
                    prev[next_index] = Some(node.as_id());
                }
            }
        }
        Err(UnfeasibleError)
    }
}

impl<Config, State, Resource> Construct<Config, State, Resource> for DijkstraSolver {
    fn construct<'a>(_config: &'a Config, _state: &'a State, _resource: &'a mut Resource) -> Self {
        Self
    }
}

impl_deconstruct_with_default!(DijkstraSolver);
