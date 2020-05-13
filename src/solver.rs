use core::cmp::Reverse;
use core::fmt::Debug;
use core::marker::PhantomData;

use generic_array::GenericArray;
use heap::BinaryHeap;
use heapless::{consts::*, ArrayLength, Vec};
use itertools::repeat_n;
use num::{Bounded, Saturating};

use super::administrator;

pub struct Solver<Node, SearchNode, Max> {
    start: Node,
    goal: Node,
    _node_max: PhantomData<fn() -> Max>,
    _search_node: PhantomData<fn() -> SearchNode>,
}

impl<Node, SearchNode, Max> Solver<Node, SearchNode, Max> {
    pub fn new(start: Node, goal: Node) -> Self {
        Self {
            start,
            goal,
            _node_max: PhantomData,
            _search_node: PhantomData,
        }
    }
}

impl<Node, SearchNode, Cost, Graph, Max> administrator::Solver<Node, SearchNode, Cost, Graph>
    for Solver<Node, SearchNode, Max>
where
    Max: ArrayLength<Node>
        + ArrayLength<SearchNode>
        + ArrayLength<Cost>
        + ArrayLength<Reverse<Cost>>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<(SearchNode, Reverse<Cost>)>
        + ArrayLength<Option<Node>>
        + ArrayLength<Option<usize>>
        + typenum::Unsigned,
    Node: Ord + Copy + Debug + Into<usize>,
    SearchNode: Ord + Copy + Debug + Into<usize>,
    Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Graph: administrator::Graph<Node, Cost>,
{
    type Nodes = Vec<Node, Max>;
    type SearchNodes = Vec<SearchNode, U4>;

    fn next_node_candidates(&self, current: SearchNode, graph: &Graph) -> Option<Self::SearchNodes>
    where
        Graph: administrator::Graph<SearchNode, Cost>
            + administrator::GraphConverter<Node, SearchNode>,
    {
        let shortest_path = self.compute_shortest_path(graph)?;
        let checker_nodes = graph.convert_to_checker_nodes(shortest_path);

        let candidates = graph
            .successors(current)
            .into_iter()
            .collect::<Vec<(SearchNode, Cost), U4>>();

        let mut dists = repeat_n(Cost::max_value(), Max::USIZE).collect::<GenericArray<_, Max>>();
        let mut heap = BinaryHeap::<SearchNode, Reverse<Cost>, Max>::new();
        for node in checker_nodes {
            heap.push(node, Reverse(Cost::min_value())).unwrap();
            dists[node.into()] = Cost::min_value();
        }
        while let Some((node, Reverse(cost))) = heap.pop() {
            if candidates.iter().any(|&(cand, _)| cand == node) {
                continue;
            }
            for (next, edge_cost) in graph.predecessors(node) {
                let next_cost = cost.saturating_add(edge_cost);
                if dists[next.into()] > next_cost {
                    dists[next.into()] = next_cost;
                    heap.push_or_update(next, Reverse(next_cost)).unwrap();
                }
            }
        }

        let mut candidates = candidates
            .into_iter()
            .map(|(node, cost)| (node, cost.saturating_add(dists[node.into()])))
            .filter(|&(_, cost)| cost == Cost::max_value())
            .collect::<Vec<(SearchNode, Cost), U4>>();

        if candidates.is_empty() {
            return None;
        }

        candidates.sort_by_key(|&(_, cost)| cost);
        Some(candidates.into_iter().map(|(node, _)| node).collect())
    }

    fn compute_shortest_path(&self, graph: &Graph) -> Option<Self::Nodes> {
        let mut dists = repeat_n(Cost::max_value(), Max::USIZE).collect::<GenericArray<_, Max>>();
        dists[self.start.into()] = Cost::min_value();

        let mut prev = repeat_n(None, Max::USIZE).collect::<GenericArray<Option<Node>, Max>>();

        let mut heap = BinaryHeap::<Node, Reverse<Cost>, Max>::new();
        heap.push(self.start, Reverse(Cost::min_value())).unwrap();

        while let Some((node, Reverse(cost))) = heap.pop() {
            if node == self.goal {
                break;
            }
            for (next, edge_cost) in graph.successors(node) {
                let next_cost = cost.saturating_add(edge_cost);
                if next_cost < dists[next.into()] {
                    dists[next.into()] = next_cost;
                    heap.push_or_update(next, Reverse(next_cost)).unwrap();
                    prev[next.into()] = Some(node);
                }
            }
        }
        let mut current = prev[self.goal.into()]?;
        let mut path = Vec::new();
        path.push(self.goal).unwrap();
        path.push(current).unwrap();
        while let Some(next) = prev[current.into()] {
            path.push(next).unwrap();
            current = next;
        }
        let len = path.len();
        //reverse
        for i in 0..len / 2 {
            path.swap(i, len - i - 1);
        }
        Some(path)
    }
}

#[cfg(test)]
mod tests {
    use super::administrator::Graph;
    use super::Solver;
    use heapless::consts::*;

    struct IGraph {
        n: usize,
        mat: Vec<Vec<Option<usize>>>,
    }

    impl IGraph {
        fn new(n: usize, edges: &[(usize, usize, usize)]) -> Self {
            let mut mat = vec![vec![None; n]; n];
            for &(src, dst, cost) in edges {
                mat[src][dst] = Some(cost);
            }
            Self { n: n, mat: mat }
        }
    }

    impl Graph<usize, usize> for IGraph {
        type Edges = Vec<(usize, usize)>;

        fn successors(&self, node: usize) -> Self::Edges {
            let mut result = Vec::new();
            for i in 0..self.n {
                if let Some(cost) = self.mat[node][i] {
                    result.push((i, cost));
                }
            }
            result
        }

        fn predecessors(&self, node: usize) -> Self::Edges {
            let mut result = Vec::new();
            for i in 0..self.n {
                if let Some(cost) = self.mat[i][node] {
                    result.push((i, cost));
                }
            }
            result
        }
    }

    #[test]
    fn test_compute_shortest_path() {
        let edges = [
            (0, 1, 2),
            (0, 2, 1),
            (1, 3, 1),
            (2, 3, 3),
            (3, 4, 2),
            (3, 5, 5),
            (3, 6, 4),
            (5, 6, 3),
            (5, 7, 7),
            (7, 8, 1),
        ];
        let start = 0;
        let goal = 8;
        let n = 9;

        let graph = IGraph::new(n, &edges);

        let solver = Solver::<usize, usize, U9>::new(start, goal);

        let path = crate::administrator::Solver::compute_shortest_path(&solver, &graph);
        let expected = [0, 1, 3, 5, 7, 8];

        assert!(path.is_some());
        assert_eq!(path.unwrap().as_ref(), expected);
    }
}
