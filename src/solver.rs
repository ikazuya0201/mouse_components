use core::ops::Add;

use generic_array::GenericArray;
use heapless::{
    binary_heap::{BinaryHeap, Min},
    ArrayLength, Vec,
};
use num::Bounded;
use typenum::Unsigned;

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
            + ArrayLength<Cost>
            + Unsigned,
    {
        let mut heap = BinaryHeap::<(Cost, Node), L, Min>::new();

        let mut dist = core::iter::repeat(Cost::max_value())
            .take(L::to_usize())
            .collect::<GenericArray<Cost, L>>();

        let mut prev = core::iter::repeat(Node::default())
            .take(L::to_usize())
            .collect::<GenericArray<Node, L>>();

        heap.push((Cost::min_value(), start)).unwrap();
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
            for node in rpath {
                path.push(node).unwrap();
            }
            path
        };

        while let Some((cost, node)) = heap.pop() {
            for &goal in goals {
                if node != goal {
                    continue;
                }
                return construct_path(goal, prev);
            }
            for &(next, c) in &graph.neighbors::<L>(node) {
                if dist[next.into()] > cost + c {
                    let ncost = cost + c;
                    dist[next.into()] = ncost;
                    prev[next.into()] = node;
                    heap.push((ncost, next)).unwrap();
                }
            }
        }
        //panic if goal is unreachable
        unreachable!()
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
        + ArrayLength<Cost>
        + Unsigned,
    GL: ArrayLength<Node>,
{
    fn start(&self) -> Node {
        self.start
    }

    fn solve(
        &self,
        current: Node,
        graph: &Graph,
    ) -> (Vec<Node, L>, fn(fn(Direction) -> bool) -> Direction) {
        let path = self.find_shortest_path(self.start, &self.goals, graph);
        let dummy = |f: fn(Direction) -> bool| Direction::North;
        (path, dummy)
    }
}
