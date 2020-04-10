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

        let dummy = |f: fn(Direction) -> bool| Direction::North;
        Some((Vec::new(), dummy))
    }
}
