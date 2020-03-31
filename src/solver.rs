use core::ops::Add;

use generic_array::GenericArray;
use heapless::{
    binary_heap::{BinaryHeap, Min},
    ArrayLength, Vec,
};
use num::Bounded;
use typenum::Unsigned;

use crate::operator;

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
{
    pub fn new(start: Node, goals: &[Node]) -> Self {
        let mut goal_vec = Vec::new();
        for &goal in goals {
            goal_vec.push(goal);
        }
        Self {
            start: start,
            goals: goal_vec,
        }
    }

    fn find_shortest_path<Cost, Direction, Graph, L>(
        &self,
        start: Node,
        goals: &[Node],
        graph: &Graph,
    ) -> Vec<Node, L>
    where
        Graph: operator::Graph<Node, Cost, Direction>,
        Cost: PartialOrd + Ord + Clone + Add<Output = Cost> + Bounded,
        Node: PartialOrd + Ord + Default + Clone + Into<usize>,
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

        heap.push((Cost::min_value(), start));
        dist[start.into()] = Cost::min_value();

        let construct_path = |goal: Node| {
            let mut rpath = Vec::<Node, L>::new();
            rpath.push(goal);
            let mut current = goal;
            while current != self.start {
                let next = prev[current.into()];
                rpath.push(next);
                current = next;
            }
            let mut path = Vec::new();
            for node in rpath {
                path.push(node);
            }
            path
        };

        while let Some((cost, node)) = heap.pop() {
            for &goal in goals {
                if node != goal {
                    continue;
                }
                return construct_path(goal);
            }
            for &(next, c) in &graph.neighbors::<L>(node) {
                if dist[next.into()] > cost + c {
                    let ncost = cost + c;
                    dist[next.into()] = ncost;
                    prev[next.into()] = node;
                    heap.push((ncost, next));
                }
            }
        }
        Vec::new()
    }
}

impl<Node, Cost, Direction, Graph, L, GL> operator::Solver<Node, Cost, Direction, Graph, L>
    for Solver<Node, GL>
where
    Graph: operator::Graph<Node, Cost, Direction>,
    Cost: PartialOrd + Ord + Clone + Add<Output = Cost> + Bounded,
    Node: PartialOrd + Ord + Default + Clone + Into<usize>,
    L: ArrayLength<Node> + ArrayLength<(Cost, Node)> + ArrayLength<Cost> + Unsigned,
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
    }
}
