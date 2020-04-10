mod path_computer;

use core::cmp::Reverse;
use core::fmt::Debug;

use generic_array::GenericArray;
use heapless::{ArrayLength, Vec};
use num::{Bounded, Saturating};

use crate::{direction, operator};
use direction::Direction;
use path_computer::PathComputer;

pub struct Solver<Node, Cost, L>
where
    L: ArrayLength<Cost>
        + ArrayLength<Option<usize>>
        + ArrayLength<(Node, Cost)>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<Node>,
{
    start: Node,
    goal: Node,
    path_computer: PathComputer<Node, Cost, L>,
}

impl<Node, Cost, L> Solver<Node, Cost, L>
where
    Node: Into<usize> + Clone + Copy + Debug + Eq,
    Cost: Clone + Copy + Ord + Default + Bounded + Debug + Saturating,
    L: ArrayLength<Cost>
        + ArrayLength<Option<usize>>
        + ArrayLength<(Node, Cost)>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<bool>
        + ArrayLength<Node>,
{
    pub fn new<Graph>(start: Node, goal: Node, graph: &Graph) -> Self
    where
        Graph: operator::Graph<Node, Cost>,
    {
        Self {
            start: start,
            goal: goal,
            path_computer: PathComputer::new(start, goal, graph),
        }
    }

    pub fn get_target_nodes<Graph>(&self, path: &[Node], graph: &Graph) -> GenericArray<bool, L>
    where
        Graph: operator::CheckableGraph<Node, Cost>,
    {
        let mut is_target_node = GenericArray::default();
        for i in 0..path.len() - 1 {
            if !graph.is_checked((path[i], path[i + 1])) {
                for node in graph.unchecked_edge_to_target_nodes((path[i], path[i + 1])) {
                    is_target_node[node.into()] = true;
                }
            }
        }
        is_target_node
    }
}

impl<Node, Cost, Graph, L> operator::Solver<Node, Cost, Direction, Graph, L>
    for Solver<Node, Cost, L>
where
    Graph: operator::DirectionalGraph<Node, Cost, Direction> + operator::CheckableGraph<Node, Cost>,
    Node: Into<usize> + Clone + Copy + Debug + Eq,
    Cost: Clone + Copy + Ord + Default + Bounded + Debug + Saturating,
    L: ArrayLength<Cost>
        + ArrayLength<Option<usize>>
        + ArrayLength<(Node, Cost)>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<bool>
        + ArrayLength<Node>,
{
    fn start(&self) -> Node {
        self.start
    }

    fn solve(
        &self,
        current: Node,
        graph: &Graph,
    ) -> Option<(Vec<Node, L>, fn(fn(Direction) -> bool) -> Direction)> {
        let shortest_path = self.path_computer.get_shortest_path(graph);

        let is_target_node = self.get_target_nodes(&shortest_path, graph);

        let dummy = |f: fn(Direction) -> bool| Direction::North;
        Some((Vec::new(), dummy))
    }
}
