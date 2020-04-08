mod path_computer;

use core::ops::Add;

use generic_array::GenericArray;
use heap::BinaryHeap;
use heapless::{ArrayLength, Vec};
use num::Bounded;

use crate::{direction, operator};
use direction::Direction;

pub struct Solver<Node, GL>
where
    GL: ArrayLength<Node>,
{
    start: Node,
    goals: Vec<Node, GL>,
}

impl<Node, GL> Solver<Node, GL>
where
    GL: ArrayLength<Node>,
    Node: Clone + Copy + core::fmt::Debug,
{
    pub fn new(start: Node, goals: &[Node]) -> Self {
        let mut goal_vec = Vec::new();
        for &goal in goals {
            goal_vec.push(goal).unwrap();
        }
        Self {
            start: start,
            goals: goal_vec,
        }
    }

    fn find_shortest_path<Cost, Graph, L>(
        &self,
        start: Node,
        goals: &[Node],
        graph: &Graph,
    ) -> Vec<Node, L>
    where
        Graph: operator::Graph<Node, Cost, Direction>,
        Cost: PartialOrd + Ord + Clone + Copy + Add<Output = Cost> + Bounded + core::fmt::Debug,
        Node: PartialOrd + Ord + Default + Clone + Copy + Into<usize> + core::fmt::Debug,
        L: ArrayLength<Node>
            + ArrayLength<(Cost, Node)>
            + ArrayLength<(Node, Cost)>
            + ArrayLength<(Node, Node)>
            + ArrayLength<Cost>
            + ArrayLength<Option<usize>>,
    {
        let mut heap = BinaryHeap::<Node, Cost, L>::new();

        let mut dist = core::iter::repeat(Cost::max_value())
            .take(L::to_usize())
            .collect::<GenericArray<Cost, L>>();

        let mut prev = core::iter::repeat(Node::default())
            .take(L::to_usize())
            .collect::<GenericArray<Node, L>>();

        heap.push(start, Cost::min_value()).unwrap();
        dist[start.into()] = Cost::min_value();

        let construct_path = |goal: Node, prev: GenericArray<Node, L>| {
            let mut rpath = Vec::<Node, L>::new();
            rpath.push(goal).unwrap();
            let mut current = goal;
            while current != self.start {
                let next = prev[current.into()];
                rpath.push(next).unwrap();
                current = next;
            }
            let mut path = Vec::new();
            let len = rpath.len();
            for i in 0..len {
                path.push(rpath[len - i - 1]).unwrap();
            }
            path
        };

        while let Some((node, cost)) = heap.pop() {
            for &goal in goals {
                if node != goal {
                    continue;
                }
                return construct_path(goal, prev);
            }
            for &(next, c) in &graph.successors::<L>(node) {
                if dist[next.into()] > cost + c {
                    let ncost = cost + c;
                    dist[next.into()] = ncost;
                    prev[next.into()] = node;
                    heap.push_or_update(next, ncost).unwrap();
                }
            }
        }
        //panic if goal is unreachable
        unreachable!()
    }

    ///This function finds an unchecked node, which is a endpoint of an unchecked edge.
    ///Multiple unchecked nodes could exist in given path,
    ///but in terms of calculation speed, consider only one node.
    fn find_first_unchecked_node<Cost, Direction, Graph, L>(
        &self,
        path: Vec<Node, L>,
        graph: &Graph,
    ) -> Option<Node>
    where
        Graph: operator::Graph<Node, Cost, Direction>,
        L: ArrayLength<(Node, Node)> + ArrayLength<Node>,
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

impl<Node, Cost, Graph, L, GL> operator::Solver<Node, Cost, Direction, Graph, L>
    for Solver<Node, GL>
where
    Graph: operator::Graph<Node, Cost, Direction>,
    Cost: PartialOrd + Ord + Clone + Copy + Add<Output = Cost> + Bounded + core::fmt::Debug,
    Node: PartialOrd + Ord + Default + Clone + Copy + Into<usize> + core::fmt::Debug,
    L: ArrayLength<Node>
        + ArrayLength<(Cost, Node)>
        + ArrayLength<(Node, Cost)>
        + ArrayLength<(Node, Node)>
        + ArrayLength<Cost>
        + ArrayLength<Option<usize>>,
    GL: ArrayLength<Node>,
{
    fn start(&self) -> Node {
        self.start
    }

    fn solve(
        &self,
        current: Node,
        graph: &Graph,
    ) -> Option<(Vec<Node, L>, fn(fn(Direction) -> bool) -> Direction)> {
        let shortest_path: Vec<Node, L> = self.find_shortest_path(self.start, &self.goals, graph);
        let unchecked_node =
            self.find_first_unchecked_node::<Cost, Direction, Graph, L>(shortest_path, graph)?;

        let path: Vec<Node, L> = self.find_shortest_path(current, &[unchecked_node], graph);

        let dummy = |f: fn(Direction) -> bool| Direction::North;
        Some((Vec::new(), dummy))
    }
}
