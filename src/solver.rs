use core::cmp::Reverse;
use core::fmt::Debug;
use core::marker::PhantomData;

use generic_array::GenericArray;
use heap::BinaryHeap;
use heapless::{consts::*, ArrayLength, Vec};
use num::{Bounded, Saturating};

use super::administrator;
use crate::utils::itertools::repeat_n;

pub struct Solver<Node, SearchNode, Max, GSize>
where
    GSize: ArrayLength<Node>,
{
    start: Node,
    goals: GenericArray<Node, GSize>,
    _node_max: PhantomData<fn() -> Max>,
    _search_node: PhantomData<fn() -> SearchNode>,
}

impl<Node, SearchNode, Max, GSize> Solver<Node, SearchNode, Max, GSize>
where
    GSize: ArrayLength<Node>,
{
    pub fn new(start: Node, goals: GenericArray<Node, GSize>) -> Self {
        Self {
            start,
            goals,
            _node_max: PhantomData,
            _search_node: PhantomData,
        }
    }
}

impl<Node, SearchNode, Cost, Graph, Max, GSize> administrator::Solver<Node, SearchNode, Cost, Graph>
    for Solver<Node, SearchNode, Max, GSize>
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
    GSize: ArrayLength<Node>,
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
            .filter(|&(_, cost)| cost < Cost::max_value())
            .collect::<Vec<(SearchNode, Cost), U4>>();

        if candidates.is_empty() {
            return None;
        }

        candidates.sort_unstable_by_key(|elem| elem.1);
        Some(candidates.into_iter().map(|(node, _)| node).collect())
    }

    fn compute_shortest_path(&self, graph: &Graph) -> Option<Self::Nodes> {
        let mut dists = repeat_n(Cost::max_value(), Max::USIZE).collect::<GenericArray<_, Max>>();
        dists[self.start.into()] = Cost::min_value();

        let mut prev = repeat_n(None, Max::USIZE).collect::<GenericArray<Option<Node>, Max>>();

        let mut heap = BinaryHeap::<Node, Reverse<Cost>, Max>::new();
        heap.push(self.start, Reverse(Cost::min_value())).unwrap();

        let construct_path = |goal: Node, prev: GenericArray<_, _>| {
            let mut current = prev[goal.into()]?;
            let mut path = Vec::new();
            path.push(goal).unwrap();
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
        };

        while let Some((node, Reverse(cost))) = heap.pop() {
            for &goal in &self.goals {
                if node == goal {
                    return construct_path(goal, prev);
                }
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
        None
    }
}

#[cfg(test)]
mod tests {
    use super::administrator::Graph;
    use super::Solver;
    use crate::maze::MazeBuilder;

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
            Self { n, mat }
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
        use generic_array::arr;

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
        let goals = arr![usize; 8];
        let n = 9;

        let graph = IGraph::new(n, &edges);

        let solver = Solver::<usize, usize, U9, _>::new(start, goals);

        let path = crate::administrator::Solver::compute_shortest_path(&solver, &graph);
        let expected = [0, 1, 3, 5, 7, 8];

        assert!(path.is_some());
        assert_eq!(path.unwrap().as_ref(), expected);
    }

    macro_rules! next_node_candidates_tests {
        ($($name: ident: $value: expr,)*) => {
            $(
                #[test]
                fn $name() {
                    use generic_array::arr;
                    use crate::maze::{AbsoluteDirection::*, WallPosition, WallDirection::*, NodeId, SearchNodeId};
                    use crate::utils::math::MathFake;
                    use crate::prelude::*;
                    use crate::data_types::Pattern;

                    fn cost(pattern: Pattern) -> u16 {
                        use Pattern::*;

                        match pattern {
                            Straight(x) => 10 * x,
                            StraightDiagonal(x) => 7 * x,
                            Search90 => 8,
                            FastRun45 => 12,
                            FastRun90 => 15,
                            FastRun135 => 20,
                            FastRun180 => 25,
                            FastRunDiagonal90 => 15,
                            SpinBack => 15,
                        }
                    }

                    let new = |x, y, dir| {
                        NodeId::<U4>::new(x, y, dir)
                            .unwrap_or_else(|| panic!("Failed to create node id: {:?}", (x,y,dir)))
                    };
                    let new_search = |x, y, dir| {
                        SearchNodeId::<U4>::new(x, y, dir)
                            .unwrap_or_else(|| panic!("Failed to create search node id: {:?}", (x,y,dir)))
                    };
                    let new_wall = |x, y, z| {
                        WallPosition::<U4>::new(x, y, z)
                            .unwrap_or_else(|| panic!("Failed to create wall position: {:?}", (x,y,z)))
                    };

                    let start = new(0, 0, North);
                    let goals = arr![NodeId<U4>; new(2,0,West), new(2,0,South)];

                    let solver = Solver::<NodeId<U4>, SearchNodeId<U4>, U256, _>::new(start, goals);

                    let (input, expected) = $value;
                    let current = input.0;
                    let walls = input.1;

                    let current = new_search(current.0, current.1, current.2);
                    let maze = MazeBuilder::new().costs(cost).build::<U4, MathFake>();
                    for (wall, exists) in walls.into_iter().map(|wall| (new_wall(wall.0,wall.1,wall.2), wall.3)) {
                        maze.check_wall(wall, exists);
                    }
                    let expected = expected.into_iter().map(|input| new_search(input.0, input.1, input.2)).collect::<Vec<_>>();
                    let candidates = solver.next_node_candidates(current, &maze);
                    assert_eq!(candidates.expect("Failed to unwrap candidates"), expected.as_slice());
                }
            )*
        }
    }

    next_node_candidates_tests! {
        test_next_node_candidates1: (
            (
                (0,1,North),
                vec![
                    (0,0,Right,true),
                    (0,0,Up,false),
                ],
            ),
            vec![
                (1,2,East),
                (0,3,North),
                (0,1,South),
            ],
        ),
        test_next_node_candidates2: (
            (
                (1,2,East),
                vec![
                    (0,0,Right,true),
                    (0,0,Up,false),
                    (0,1,Right,false),
                    (0,1,Up,false),
                ],
            ),
            vec![
                (2,1,South),
                (2,3,North),
                (3,2,East),
                (1,2,West),
            ],
        ),
        test_next_node_candidates3: (
            (
                (6,5,South),
                vec![
                    (0,1,Right,true),
                    (0,1,Up,false),
                    (0,2,Right,true),
                    (0,2,Up,false),
                    (0,3,Right,false),
                    (1,2,Up,false),
                    (1,3,Right,false),
                    (2,2,Right,false),
                    (2,2,Up,true),
                    (2,3,Right,false),
                    (3,1,Up,true),
                    (3,2,Up,false),
                ],
            ),
            vec![
                (5,4,West),
                (6,5,North),
            ],
        ),
    }
}
