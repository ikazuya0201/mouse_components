mod path_computer;

use core::cmp::Reverse;
use core::fmt::Debug;

use generic_array::GenericArray;
use heapless::{ArrayLength, Vec};
use heap::BinaryHeap;
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

    fn get_target_nodes<Graph>(&self, path: &[Node], graph: &Graph) -> GenericArray<bool, L>
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

    fn compute_shortest_path<Graph>(&self, start: Node, is_target: &[bool], graph: &Graph) -> Vec<Node, L> 
    where Graph: operator::Graph<Node, Cost>,
          L: ArrayLength<Cost> + ArrayLength<Option<Node>>,
    {
        let mut heap = BinaryHeap::<Node, Reverse<Cost>, L>::new();
        let mut dist = GenericArray::<Cost, L>::default();
        let mut prev = GenericArray::<Option<Node>,L>::default();
        for i in 0..L::to_usize() {
            dist[i] = Cost::max_value();
        }

        heap.push(start, Reverse(Cost::min_value())).unwrap();
        dist[start.into()] = Cost::min_value();

        let construct_path = |goal: Node, prev: GenericArray<Option<Node>,L>| {
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
                path.push(rpath[rpath.len()-i-1]).unwrap();
            }
            path
        };

        while let Some((node, Reverse(cost))) = heap.pop() {
            if is_target[node.into()] {
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

        let is_target = self.get_target_nodes(&shortest_path, graph);

        let path_to_target = self.compute_shortest_path(current, &is_target, graph);

        let dummy = |f: fn(Direction) -> bool| Direction::North;
        Some((Vec::new(), dummy))
    }
}
