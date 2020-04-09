use core::cmp::Reverse;
use core::fmt::Debug;
use core::ops::Add;

use generic_array::{ArrayLength, GenericArray};
use heap::BinaryHeap;
use heapless::Vec;
use num::{Bounded, Saturating};

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
    Cost: Clone + Copy + Ord + Default + Bounded + Debug + Saturating,
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
            self.g[i] = Cost::max_value();
            self.rhs[i] = Cost::max_value();
        }
        self.rhs[self.start.into()] = Cost::min_value();
        self.heap
            .push(self.start, Reverse(self.calculate_value(self.start)))
            .unwrap();
    }

    fn calculate_value(&self, node: Node) -> Cost {
        self.g[node.into()].min(self.rhs[node.into()])
    }

    pub fn update_node<Direction, Graph>(&mut self, node: Node, graph: &Graph)
    where
        Graph: operator::Graph<Node, Cost, Direction>,
    {
        if self.start != node {
            let mut min = Cost::max_value();
            for (pred, cost) in graph.predecessors::<L>(node) {
                min = min.min(self.g[pred.into()].saturating_add(cost));
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

    fn compute_shortest_path<Direction, Graph>(&mut self, graph: &Graph) -> bool
    where
        Graph: operator::Graph<Node, Cost, Direction>,
    {
        let mut is_updated = false;
        while let Some((node, Reverse(value))) = self.heap.pop() {
            if self.calculate_value(self.goal) <= value
                && self.rhs[self.goal.into()] == self.g[self.goal.into()]
            {
                self.heap.push(node, Reverse(value)).unwrap();
                break;
            }
            is_updated = true;
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
        is_updated
    }

    pub fn update_shortest_path<Direction, Graph>(&mut self, graph: &Graph)
    where
        Graph: operator::Graph<Node, Cost, Direction>,
    {
        if !self.compute_shortest_path(graph) {
            return;
        }
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
                let new_cost = self.g[pred.into()].saturating_add(cost);
                if min_cost > new_cost {
                    min_cost = new_cost;
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
        self.shortest_path = rpath;
    }

    pub fn get_shortest_path(&self) -> &Vec<Node, L> {
        &self.shortest_path
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use heapless::consts::*;

    struct Graph {
        mat: GenericArray<GenericArray<Option<usize>, U8>, U8>,
    }

    impl Graph {
        fn new(edges: &[(usize, usize, usize)]) -> Self {
            let mut mat = GenericArray::<GenericArray<Option<usize>, U8>, U8>::default();
            for &(src, dst, cost) in edges {
                mat[src][dst] = Some(cost);
            }
            Self { mat: mat }
        }
    }

    impl operator::Graph<usize, usize, bool> for Graph {
        fn successors<L: ArrayLength<(usize, usize)>>(
            &self,
            node: usize,
        ) -> Vec<(usize, usize), L> {
            let mut succ = Vec::new();
            for i in 0..L::to_usize() {
                if let Some(cost) = self.mat[node][i] {
                    succ.push((i, cost)).unwrap();
                }
            }
            succ
        }

        fn predecessors<L: ArrayLength<(usize, usize)>>(
            &self,
            node: usize,
        ) -> Vec<(usize, usize), L> {
            let mut pred = Vec::new();
            for i in 0..L::to_usize() {
                if let Some(cost) = self.mat[i][node] {
                    pred.push((i, cost)).unwrap();
                }
            }
            pred
        }

        fn is_checked(&self, _edge: (usize, usize)) -> bool {
            true
        }

        fn edge_direction(&self, _edge: (usize, usize)) -> bool {
            true
        }
        fn separate_to_unit_edges<L: ArrayLength<(usize, usize)>>(
            &self,
            _edge: (usize, usize),
        ) -> Vec<(usize, usize), L> {
            Vec::new()
        }
    }

    impl Graph {
        fn update_edge(&mut self, src: usize, dst: usize, cost: usize) {
            self.mat[src][dst] = Some(cost);
        }
    }

    #[test]
    fn shortest_path() {
        let start = 0usize;
        let goal = 6usize;
        let directed_edges = [
            (0, 1, 10),
            (0, 2, 8),
            (1, 3, 5),
            (2, 3, 2),
            (3, 4, 9),
            (3, 5, 7),
            (4, 6, 2),
            (5, 6, 3),
        ];

        let mut graph = Graph::new(&directed_edges);
        let mut computer = PathComputer::<usize, usize, U8>::new(start, goal);

        computer.update_shortest_path(&graph);
        assert_eq!(computer.get_shortest_path().as_ref(), [0, 2, 3, 5, 6]);

        graph.update_edge(3, 4, 1);
        computer.update_node(4, &graph);
        computer.update_shortest_path(&graph);
        assert_eq!(computer.get_shortest_path().as_ref(), [0, 2, 3, 4, 6]);
    }
}
