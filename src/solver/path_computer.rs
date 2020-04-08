use core::cmp::Reverse;
use core::fmt::Debug;
use core::ops::Add;

use generic_array::{ArrayLength, GenericArray};
use heap::BinaryHeap;
use heapless::Vec;
use num::Bounded;

use super::operator;

pub struct PathComputer<Node, Cost, L>
where
    L: ArrayLength<Cost>
        + ArrayLength<Option<usize>>
        + ArrayLength<(Node, Cost)>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<Node>,
{
    start: Node,
    goal: Node,
    g: GenericArray<Cost, L>,
    rhs: GenericArray<Cost, L>,
    heap: BinaryHeap<Node, Reverse<Cost>, L>,
    shortest_path: Vec<Node, L>,
}

impl<Node, Cost, L> PathComputer<Node, Cost, L>
where
    Node: Into<usize> + Clone + Copy + Debug + Eq,
    Cost: Clone + Copy + Ord + Default + Bounded + Debug + Add<Output = Cost>,
    L: ArrayLength<Cost>
        + ArrayLength<Option<usize>>
        + ArrayLength<(Node, Cost)>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<Node>,
{
    pub fn new(start: Node, goal: Node) -> Self {
        let heap = BinaryHeap::new();
        let g = GenericArray::default();
        let rhs = GenericArray::default();

        let mut computer = Self {
            start: start,
            goal: goal,
            g: g,
            rhs: rhs,
            heap: heap,
            shortest_path: Vec::new(),
        };
        computer.initialize();
        computer
    }

    fn initialize(&mut self) {
        for i in 0..L::to_usize() {
            self.g[i] = Cost::min_value();
            self.rhs[i] = Cost::min_value();
        }
        self.rhs[self.start.into()] = Cost::min_value();
        self.heap
            .push(self.start, Reverse(self.calculate_value(self.start)))
            .unwrap();
    }

    fn calculate_value(&self, node: Node) -> Cost {
        self.g[node.into()].min(self.rhs[node.into()])
    }

    fn update_node<Direction, Graph>(&mut self, node: Node, graph: &Graph)
    where
        Graph: operator::Graph<Node, Cost, Direction>,
    {
        if self.start != node {
            let mut min = Cost::max_value();
            for (pred, cost) in graph.predecessors::<L>(node) {
                if self.g[pred.into()] == Cost::max_value() {
                    continue;
                }
                min = min.min(self.g[pred.into()] + cost);
            }
            self.rhs[node.into()] = min;
        }
        self.heap.remove(node).ok();
        if self.g[node.into()] != self.rhs[node.into()] {
            self.heap
                .push(node, Reverse(self.calculate_value(node)))
                .unwrap();
        }
    }

    pub fn compute_shortest_path<Direction, Graph>(&mut self, graph: &Graph)
    where
        Graph: operator::Graph<Node, Cost, Direction>,
    {
        let mut is_updated = false;
        while (self.heap.peek().unwrap().1).0 < self.calculate_value(self.goal)
            || self.rhs[self.goal.into()] != self.g[self.goal.into()]
        {
            is_updated = true;
            let (node, _) = self.heap.pop().unwrap();
            if self.g[node.into()] > self.rhs[node.into()] {
                self.g[node.into()] = self.rhs[node.into()];
            } else {
                self.g[node.into()] = Cost::max_value();
                self.update_node(node, graph);
            }
            for (succ, _) in graph.successors::<L>(node) {
                self.update_node(succ, graph);
            }
        }
        if is_updated {
            self.shortest_path = self.update_shortest_path(graph);
        }
    }

    fn update_shortest_path<Direction, Graph>(&self, graph: &Graph) -> Vec<Node, L>
    where
        Graph: operator::Graph<Node, Cost, Direction>,
    {
        let mut path = Vec::<Node, L>::new();
        let mut current = self.goal;
        path.push(current).unwrap();
        while current != self.start {
            let mut min_cost = Cost::max_value();
            let mut min_node = current;
            for (pred, cost) in graph.predecessors::<L>(current) {
                if self.g[pred.into()] == Cost::max_value() {
                    continue;
                }
                if min_cost > self.g[pred.into()] + cost {
                    min_cost = self.g[pred.into()] + cost;
                    min_node = pred;
                }
            }
            current = min_node;
            path.push(current).unwrap();
        }
        let mut rpath = Vec::new();
        for i in 0..path.len() {
            rpath.push(path[path.len() - i - 1]).unwrap();
        }
        rpath
    }

    pub fn get_shortest_path(&self) -> &Vec<Node, L> {
        &self.shortest_path
    }
}
