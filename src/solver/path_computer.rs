use core::cmp::Reverse;
use core::fmt::Debug;

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
    pub fn new<Graph>(start: Node, goal: Node, graph: &Graph) -> Self
    where
        Graph: operator::Graph<Node, Cost>,
    {
        let heap = BinaryHeap::new();
        let g = GenericArray::default();
        let rhs = GenericArray::default();

        let mut computer = Self {
            start: start,
            goal: goal,
            g: g,
            rhs: rhs,
            heap: heap,
        };
        computer.initialize(graph);
        computer
    }

    pub fn start(&self) -> Node {
        self.start
    }

    fn initialize<Graph>(&mut self, graph: &Graph)
    where
        Graph: operator::Graph<Node, Cost>,
    {
        for i in 0..L::to_usize() {
            self.g[i] = Cost::max_value();
            self.rhs[i] = Cost::max_value();
        }
        self.rhs[self.start.into()] = Cost::min_value();
        self.heap
            .push(self.start, Reverse(self.calculate_value(self.start)))
            .unwrap();
        self.compute_shortest_path(graph);
    }

    fn calculate_value(&self, node: Node) -> Cost {
        self.g[node.into()].min(self.rhs[node.into()])
    }

    pub fn update_node<Graph>(&mut self, node: Node, graph: &Graph)
    where
        Graph: operator::Graph<Node, Cost>,
    {
        if self.start != node {
            let mut min = Cost::max_value();
            for (pred, cost) in graph.predecessors(node) {
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

    pub fn compute_shortest_path<Graph>(&mut self, graph: &Graph)
    where
        Graph: operator::Graph<Node, Cost>,
    {
        while let Some(&(node, Reverse(value))) = self.heap.peek() {
            if self.calculate_value(self.goal) <= value
                && self.rhs[self.goal.into()] == self.g[self.goal.into()]
            {
                break;
            }
            self.heap.pop();
            if self.g[node.into()] > self.rhs[node.into()] {
                self.g[node.into()] = self.rhs[node.into()];
            } else {
                self.g[node.into()] = Cost::max_value();
                self.update_node(node, graph);
            }
            for (succ, _) in graph.successors(node) {
                self.update_node(succ, graph);
            }
        }
    }

    pub fn get_shortest_path<Graph>(&self, graph: &Graph) -> Vec<Node, L>
    where
        Graph: operator::Graph<Node, Cost>,
    {
        let mut path = Vec::<Node, L>::new();
        let mut current = self.goal;
        path.push(current).unwrap();
        while current != self.start {
            let mut min_cost = Cost::max_value();
            let mut min_node = current;
            for (pred, cost) in graph.predecessors(current) {
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
        rpath
    }
}

impl<Node, Cost, L> Clone for PathComputer<Node, Cost, L>
where
    L: ArrayLength<Cost>
        + ArrayLength<Option<usize>>
        + ArrayLength<(Node, Cost)>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<Node>,
    Node: Clone,
    Cost: Clone,
{
    fn clone(&self) -> Self {
        Self {
            start: self.start.clone(),
            goal: self.goal.clone(),
            g: self.g.clone(),
            rhs: self.rhs.clone(),
            heap: self.heap.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use heapless::consts::*;

    struct Graph<N>
    where
        N: ArrayLength<Option<usize>> + ArrayLength<GenericArray<Option<usize>, N>>,
    {
        mat: GenericArray<GenericArray<Option<usize>, N>, N>,
    }

    impl<N> Graph<N>
    where
        N: ArrayLength<Option<usize>> + ArrayLength<GenericArray<Option<usize>, N>>,
    {
        fn new(edges: &[(usize, usize, usize)]) -> Self {
            let mut mat = GenericArray::<GenericArray<Option<usize>, N>, N>::default();
            for &(src, dst, cost) in edges {
                mat[src][dst] = Some(cost);
            }
            Self { mat: mat }
        }

        fn update_edge(&mut self, src: usize, dst: usize, cost: usize) {
            self.mat[src][dst] = Some(cost);
        }
    }

    impl<N> operator::Graph<usize, usize> for Graph<N>
    where
        N: ArrayLength<Option<usize>>
            + ArrayLength<GenericArray<Option<usize>, N>>
            + ArrayLength<(usize, usize)>,
    {
        type Edges = Vec<(usize, usize), N>;

        fn successors(&self, node: usize) -> Self::Edges {
            let mut succ = Vec::<(usize, usize), N>::new();
            for i in 0..N::to_usize() {
                if let Some(cost) = self.mat[node][i] {
                    succ.push((i, cost)).unwrap();
                }
            }
            succ
        }

        fn predecessors(&self, node: usize) -> Self::Edges {
            let mut pred = Vec::<(usize, usize), N>::new();
            for i in 0..N::to_usize() {
                if let Some(cost) = self.mat[i][node] {
                    pred.push((i, cost)).unwrap();
                }
            }
            pred
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

        let mut graph = Graph::<U8>::new(&directed_edges);
        let mut computer = PathComputer::<usize, usize, U8>::new(start, goal, &graph);

        computer.compute_shortest_path(&graph);
        assert_eq!(computer.get_shortest_path(&graph).as_ref(), [0, 2, 3, 5, 6]);

        graph.update_edge(3, 5, std::usize::MAX);
        computer.update_node(5, &graph);
        computer.compute_shortest_path(&graph);
        assert_eq!(computer.get_shortest_path(&graph).as_ref(), [0, 2, 3, 4, 6]);

        graph.update_edge(2, 3, std::usize::MAX);
        computer.update_node(3, &graph);
        computer.compute_shortest_path(&graph);
        assert_eq!(computer.get_shortest_path(&graph).as_ref(), [0, 1, 3, 4, 6]);

        graph.update_edge(0, 2, std::usize::MAX);
        computer.update_node(2, &graph);
        computer.compute_shortest_path(&graph);
        assert_eq!(computer.get_shortest_path(&graph).as_ref(), [0, 1, 3, 4, 6]);
    }
}
