mod path_computer;

use core::cmp::Reverse;
use core::fmt::Debug;

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
        + ArrayLength<Node>
        + ArrayLength<(Node, Node)>,
{
    pub fn new(start: Node, goal: Node) -> Self {
        Self {
            start: start,
            goal: goal,
            path_computer: PathComputer::new(start, goal),
        }
    }

    ///This function finds an unchecked node, which is a endpoint of an unchecked edge.
    ///Multiple unchecked nodes could exist in given path,
    ///but in terms of calculation speed, consider only one node.
    fn find_first_unchecked_node<Direction, Graph>(
        &self,
        path: Vec<Node, L>,
        graph: &Graph,
    ) -> Option<Node>
    where
        Graph: operator::Graph<Node, Cost, Direction>,
    {
        for i in 0..path.len() - 1 {
            if graph.is_checked((path[i], path[i + 1])) {
                continue;
            }
            for edge in graph.separate_to_unit_edges::<L>((path[i], path[i + 1])) {
                if !graph.is_checked(edge) {
                    return Some(edge.0);
                }
            }
        }
        None
    }
}

impl<Node, Cost, Graph, L> operator::Solver<Node, Cost, Direction, Graph, L>
    for Solver<Node, Cost, L>
where
    Graph: operator::Graph<Node, Cost, Direction>,
    Node: Into<usize> + Clone + Copy + Debug + Eq,
    Cost: Clone + Copy + Ord + Default + Bounded + Debug + Saturating,
    L: ArrayLength<Cost>
        + ArrayLength<Option<usize>>
        + ArrayLength<(Node, Cost)>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<(Node, Node)>
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
        let unchecked_node =
            self.find_first_unchecked_node::<Direction, Graph>(shortest_path, graph)?;

        let dummy = |f: fn(Direction) -> bool| Direction::North;
        Some((Vec::new(), dummy))
    }
}
