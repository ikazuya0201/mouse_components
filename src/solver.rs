mod path_computer;

use core::cmp::Reverse;
use core::fmt::Debug;

use generic_array::GenericArray;
use heap::BinaryHeap;
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

    fn get_checker_nodes<Graph>(&self, path: &[Node], graph: &Graph) -> GenericArray<bool, L>
    where
        Graph: operator::CheckableGraph<Node, Cost>,
    {
        let mut is_checker_node = GenericArray::default();
        for i in 0..path.len() - 1 {
            if !graph.is_checked((path[i], path[i + 1])) {
                for node in graph.unchecked_edge_to_checker_nodes((path[i], path[i + 1])) {
                    is_checker_node[node.into()] = true;
                }
            }
        }
        is_checker_node
    }

    fn compute_shortest_path<Graph>(
        &self,
        start: Node,
        is_goal: &[bool],
        graph: &Graph,
    ) -> Vec<Node, L>
    where
        Graph: operator::Graph<Node, Cost>,
        L: ArrayLength<Cost> + ArrayLength<Option<Node>>,
    {
        let mut heap = BinaryHeap::<Node, Reverse<Cost>, L>::new();
        let mut dist = GenericArray::<Cost, L>::default();
        let mut prev = GenericArray::<Option<Node>, L>::default();
        for i in 0..L::to_usize() {
            dist[i] = Cost::max_value();
        }

        heap.push(start, Reverse(Cost::min_value())).unwrap();
        dist[start.into()] = Cost::min_value();

        let construct_path = |goal: Node, prev: GenericArray<Option<Node>, L>| {
            let mut rpath = Vec::<Node, L>::new();
            let mut current = goal;
            rpath.push(goal).unwrap();
            while let Some(next) = prev[current.into()] {
                rpath.push(next).unwrap();
                current = next;
                if next == start {
                    break;
                }
            }
            let mut path = Vec::new();
            for i in 0..rpath.len() {
                path.push(rpath[rpath.len() - i - 1]).unwrap();
            }
            path
        };

        while let Some((node, Reverse(cost))) = heap.pop() {
            if is_goal[node.into()] {
                return construct_path(node, prev);
            }
            for (succ, scost) in graph.successors(node) {
                let ncost = cost.saturating_add(scost);
                if dist[succ.into()] > ncost {
                    dist[succ.into()] = ncost;
                    prev[succ.into()] = Some(node);
                    heap.push_or_update(succ, Reverse(ncost)).unwrap();
                }
            }
        }
        unreachable!();
    }

    fn find_first_checker_node_and_next_direction<Direction, Graph>(
        &self,
        path: &[Node],
        graph: &Graph,
    ) -> (Node, Direction)
    where
        Graph: operator::DirectionalGraph<Node, Cost, Direction>,
    {
        for i in 0..path.len() - 1 {
            if !graph.is_checked((path[i], path[i + 1])) {
                return graph.find_first_checker_node_and_next_direction((path[i], path[i + 1]));
            }
        }
        unreachable!();
    }
}

impl<Node, Cost, Graph, L> operator::Solver<Node, Cost, Direction, Graph, L>
    for Solver<Node, Cost, L>
where
    Graph: operator::DirectionalGraph<Node, Cost, Direction>,
    Node: Into<usize> + Clone + Copy + Debug + Eq,
    Cost: Clone + Copy + Ord + Default + Bounded + Debug + Saturating,
    L: ArrayLength<Cost>
        + ArrayLength<Option<usize>>
        + ArrayLength<(Node, Cost)>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<bool>
        + ArrayLength<Option<Node>>
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

        let is_checker = self.get_checker_nodes(&shortest_path, graph);

        let path_to_checker = self.compute_shortest_path(current, &is_checker, graph);

        let (checker, direction) =
            self.find_first_checker_node_and_next_direction(&path_to_checker, graph);

        let dummy = |f: fn(Direction) -> bool| Direction::North;
        Some((Vec::new(), dummy))
    }
}
