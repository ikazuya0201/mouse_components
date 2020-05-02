mod path_computer;

use core::cell::{Cell, RefCell};
use core::cmp::Reverse;
use core::fmt::Debug;
use core::marker::PhantomData;

use generic_array::GenericArray;
use heap::BinaryHeap;
use heapless::{ArrayLength, Vec};
use num::{Bounded, Saturating};

use crate::operator;
use path_computer::PathComputer;

pub struct Solver<Node, Cost, Direction, L, DL>
where
    L: ArrayLength<Cost>
        + ArrayLength<Option<usize>>
        + ArrayLength<(Node, Cost)>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<Node>,
    DL: ArrayLength<Direction>,
{
    path_computer: RefCell<PathComputer<Node, Cost, L>>,
    direction_computer: Cell<Option<DirectionComputer<Node, Direction, DL>>>,
    _direction: PhantomData<fn() -> Direction>,
    _direction_length: PhantomData<fn() -> DL>,
}

impl<Node, Cost, Direction, L, DL> Solver<Node, Cost, Direction, L, DL>
where
    Node: Into<usize> + Clone + Copy + Debug + Eq,
    Cost: Clone + Copy + Ord + Default + Bounded + Debug + Saturating,
    L: ArrayLength<Cost>
        + ArrayLength<Option<usize>>
        + ArrayLength<(Node, Cost)>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<bool>
        + ArrayLength<Node>,
    DL: ArrayLength<Direction>,
{
    pub fn new<Graph>(start: Node, goal: Node, graph: &Graph) -> Self
    where
        Graph: operator::Graph<Node, Cost>,
    {
        Self {
            path_computer: RefCell::new(PathComputer::new(start, goal, graph)),
            direction_computer: Cell::new(None),
            _direction: PhantomData,
            _direction_length: PhantomData,
        }
    }
}

impl<Node, Cost, Direction, Graph, L, DL> operator::Solver<Node, Cost, Direction, Graph>
    for Solver<Node, Cost, Direction, L, DL>
where
    Graph: operator::DirectionalGraph<Node, Cost, Direction> + Clone,
    Node: Into<usize> + Clone + Copy + Debug + Eq,
    Cost: Clone + Copy + Ord + Default + Bounded + Debug + Saturating,
    Direction: Clone + Copy + Debug,
    L: ArrayLength<Cost>
        + ArrayLength<Option<usize>>
        + ArrayLength<(Node, Cost)>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<bool>
        + ArrayLength<Option<Node>>
        + ArrayLength<Node>,
    DL: ArrayLength<Direction>,
{
    type Nodes = Vec<Node, L>;
    type Directions = Vec<Direction, DL>;

    fn start_node(&self) -> Node {
        self.path_computer.borrow().start()
    }

    fn update_node(&self, node: Node, graph: &Graph) {
        self.path_computer.borrow_mut().update_node(node, graph);
    }

    fn next_path(&self, current: Node, graph: &Graph) -> Option<Self::Nodes> {
        let shortest_path = self.path_computer.borrow().get_shortest_path(graph);

        let is_checker: GenericArray<bool, L> = get_checker_nodes(&shortest_path, graph);

        if let Some(path_to_checker) = compute_shortest_path_with_multi_goals::<Node, Cost, Graph, L>(
            current,
            &is_checker,
            graph,
        ) {
            let (checker, direction) =
                find_first_checker_node_and_next_direction(&path_to_checker, graph).unwrap();

            let path = compute_checked_shortest_path(current, checker, graph).unwrap();

            self.direction_computer
                .set(Some(DirectionComputer::new(checker, direction)));

            Some(path)
        } else {
            self.direction_computer.set(None);
            None
        }
    }

    fn last_node(&self) -> Option<Node> {
        Some(self.direction_computer.get()?.start())
    }

    fn next_direction_candidates(&self, graph: &Graph) -> Option<Self::Directions> {
        Some(
            self.direction_computer
                .get()?
                .compute_direction_candidates(&self.path_computer.borrow(), graph),
        )
    }
}

fn get_checker_nodes<Node, Cost, Graph, L>(path: &[Node], graph: &Graph) -> GenericArray<bool, L>
where
    Node: Into<usize> + Clone + Copy,
    Graph: operator::CheckableGraph<Node, Cost>,
    L: ArrayLength<bool>,
{
    let mut is_checker_node = GenericArray::default();
    for i in 0..path.len() - 1 {
        if !graph.is_checked((path[i], path[i + 1])) {
            for node in graph.convert_to_checker_nodes((path[i], path[i + 1])) {
                is_checker_node[node.into()] = true;
            }
        }
    }
    is_checker_node
}

fn compute_shortest_path_with_multi_goals<Node, Cost, Graph, L>(
    start: Node,
    is_goal: &[bool],
    graph: &Graph,
) -> Option<Vec<Node, L>>
where
    Node: Into<usize> + Clone + Copy + Debug + Eq,
    Cost: Clone + Copy + Ord + Default + Bounded + Debug + Saturating,
    Graph: operator::Graph<Node, Cost>,
    L: ArrayLength<Cost>
        + ArrayLength<Option<Node>>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<Option<usize>>
        + ArrayLength<Node>,
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
            return Some(construct_path(node, prev));
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
    None
}

fn compute_checked_shortest_path<Node, Cost, Graph, L>(
    start: Node,
    goal: Node,
    graph: &Graph,
) -> Option<Vec<Node, L>>
where
    Node: Into<usize> + Clone + Copy + Debug + Eq,
    Cost: Clone + Copy + Ord + Default + Bounded + Debug + Saturating,
    Graph: operator::CheckableGraph<Node, Cost>,
    L: ArrayLength<Cost>
        + ArrayLength<Option<Node>>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<Option<usize>>
        + ArrayLength<Node>,
{
    let mut heap = BinaryHeap::<Node, Reverse<Cost>, L>::new();
    let mut dist = GenericArray::<Cost, L>::default();
    let mut prev = GenericArray::<Option<Node>, L>::default();
    for i in 0..L::to_usize() {
        dist[i] = Cost::max_value();
    }

    heap.push(goal, Reverse(Cost::min_value())).unwrap();
    dist[goal.into()] = Cost::min_value();

    let construct_path = |goal: Node, prev: GenericArray<Option<Node>, L>| {
        let mut path = Vec::<Node, L>::new();
        let mut current = start;
        path.push(goal).unwrap();
        while let Some(next) = prev[current.into()] {
            path.push(next).unwrap();
            current = next;
            if next == goal {
                break;
            }
        }
        path
    };

    while let Some((node, Reverse(cost))) = heap.pop() {
        if node == start {
            return Some(construct_path(node, prev));
        }
        for (pred, pcost) in graph.checked_predecessors(node) {
            let ncost = cost.saturating_add(pcost);
            if dist[pred.into()] > ncost {
                dist[pred.into()] = ncost;
                prev[pred.into()] = Some(node);
                heap.push_or_update(pred, Reverse(ncost)).unwrap();
            }
        }
    }
    None
}

fn find_first_checker_node_and_next_direction<Node, Cost, Direction, Graph>(
    path: &[Node],
    graph: &Graph,
) -> Option<(Node, Direction)>
where
    Node: Clone + Copy,
    Cost: Bounded + Ord,
    Graph: operator::DirectionalGraph<Node, Cost, Direction>,
{
    for i in 0..path.len() - 1 {
        if !graph.is_checked((path[i], path[i + 1])) {
            return Some(graph.find_first_checker_node_and_next_direction((path[i], path[i + 1])));
        }
    }
    let last_node = path[path.len() - 1];
    let mut next_node = None;
    let mut min_cost = Cost::max_value();
    for (succ, cost) in graph.unchecked_successors(last_node) {
        if min_cost > cost {
            next_node = Some(succ);
            min_cost = cost;
        }
    }
    Some((last_node, graph.edge_direction((last_node, next_node?))))
}

struct DirectionComputer<Node, Direction, DL> {
    start: Node,
    first_direction: Direction,
    _direction_length: PhantomData<fn() -> DL>,
}

impl<Node, Direction, DL> DirectionComputer<Node, Direction, DL> {
    fn new(start: Node, first_direction: Direction) -> Self {
        DirectionComputer {
            start: start,
            first_direction: first_direction,
            _direction_length: PhantomData,
        }
    }

    fn start(&self) -> Node
    where
        Node: Clone,
    {
        self.start.clone()
    }

    fn compute_direction_candidates<Cost, Graph, L>(
        &self,
        path_computer: &PathComputer<Node, Cost, L>,
        graph: &Graph,
    ) -> Vec<Direction, DL>
    where
        Node: Into<usize> + Clone + Copy + Debug + Eq,
        Cost: Clone + Copy + Ord + Default + Bounded + Debug + Saturating,
        Direction: Clone + Copy + Debug,
        Graph: operator::DirectionalGraph<Node, Cost, Direction> + Clone,
        L: ArrayLength<Cost>
            + ArrayLength<Option<usize>>
            + ArrayLength<(Node, Cost)>
            + ArrayLength<(Node, Reverse<Cost>)>
            + ArrayLength<bool>
            + ArrayLength<Option<Node>>
            + ArrayLength<Node>,
        DL: ArrayLength<Direction>,
    {
        let mut directions = Vec::new();
        let mut current_direction = self.first_direction;
        let mut graph = graph.clone();
        let mut path_computer = path_computer.clone();
        loop {
            directions.push(current_direction).unwrap();
            let updated_nodes = graph.block(self.start, current_direction);
            for node in updated_nodes {
                path_computer.update_node(node, &graph);
            }
            path_computer.compute_shortest_path(&graph);
            let shortest_path = path_computer.get_shortest_path(&graph);
            let is_checker: GenericArray<bool, L> = get_checker_nodes(&shortest_path, &graph);
            if let Some(path_to_checker) = compute_shortest_path_with_multi_goals::<
                Node,
                Cost,
                Graph,
                L,
            >(self.start, &is_checker, &graph)
            {
                if path_to_checker.is_empty() {
                    break;
                }
                current_direction = graph.edge_direction((self.start, path_to_checker[0]));
            } else {
                break;
            }
        }
        directions
    }
}

impl<Node, Direction, DL> Clone for DirectionComputer<Node, Direction, DL>
where
    Node: Clone,
    Direction: Clone,
{
    fn clone(&self) -> Self {
        DirectionComputer {
            start: self.start.clone(),
            first_direction: self.first_direction.clone(),
            _direction_length: PhantomData,
        }
    }
}

impl<Node, Direction, DL> Copy for DirectionComputer<Node, Direction, DL>
where
    Node: Copy,
    Direction: Copy,
{
}

#[cfg(test)]
mod tests {
    use heapless::consts::*;

    use super::{
        compute_checked_shortest_path, compute_shortest_path_with_multi_goals, get_checker_nodes,
    };
    use crate::operator::{CheckableGraph, Graph};

    struct IGraph {
        n: usize,
        mat: Vec<Vec<Option<usize>>>,
        is_checked: Vec<Vec<bool>>,
    }

    impl IGraph {
        fn new(n: usize, edges: &[(usize, usize, usize)]) -> Self {
            let mut mat = vec![vec![None; n]; n];
            for &(src, dst, cost) in edges {
                mat[src][dst] = Some(cost);
            }
            Self {
                n: n,
                mat: mat,
                is_checked: vec![vec![true; n]; n],
            }
        }

        fn new_unchecked(n: usize, edges: &[(usize, usize, usize)]) -> Self {
            let mut graph = Self::new(n, edges);
            graph.is_checked = vec![vec![false; n]; n];
            graph
        }

        fn check(&mut self, edge: (usize, usize)) {
            self.is_checked[edge.0][edge.1] = true;
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

    impl CheckableGraph<usize, usize> for IGraph {
        type Nodes = Vec<usize>;

        fn is_checked(&self, edge: (usize, usize)) -> bool {
            self.is_checked[edge.0][edge.1]
        }

        fn convert_to_checker_nodes(&self, edge: (usize, usize)) -> Self::Nodes {
            vec![edge.0, edge.1]
        }

        fn checked_successors(&self, node: usize) -> Self::Edges {
            self.successors(node)
                .into_iter()
                .filter(|&(next, _)| self.is_checked((node, next)))
                .collect()
        }

        fn checked_predecessors(&self, node: usize) -> Self::Edges {
            self.predecessors(node)
                .into_iter()
                .filter(|&(next, _)| self.is_checked((node, next)))
                .collect()
        }

        fn unchecked_successors(&self, node: usize) -> Self::Edges {
            self.successors(node)
                .into_iter()
                .filter(|&(next, _)| !self.is_checked((node, next)))
                .collect()
        }

        fn unchecked_predecessors(&self, node: usize) -> Self::Edges {
            self.predecessors(node)
                .into_iter()
                .filter(|&(next, _)| !self.is_checked((node, next)))
                .collect()
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

        let mut is_goal = vec![false; n];
        is_goal[goal] = true;

        let graph = IGraph::new(n, &edges);

        let path = compute_shortest_path_with_multi_goals::<usize, usize, IGraph, U10>(
            start, &is_goal, &graph,
        );
        let expected = [0, 1, 3, 5, 7, 8];

        assert!(path.is_some());
        assert_eq!(path.unwrap().as_ref(), expected);
    }

    #[test]
    fn test_compute_checked_shortest_path() {
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

        let path = compute_checked_shortest_path::<usize, usize, IGraph, U10>(start, goal, &graph);
        let expected = [0, 1, 3, 5, 7, 8];

        assert!(path.is_some());
        assert_eq!(path.unwrap().as_ref(), expected);
    }

    #[test]
    fn test_get_checker_nodes() {
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

        let mut is_goal = vec![false; n];
        is_goal[goal] = true;

        let mut graph = IGraph::new_unchecked(n, &edges);
        let checked_edges = [(0, 1), (0, 2), (3, 5), (5, 7)];
        for &edge in &checked_edges {
            graph.check(edge);
        }

        let path = compute_shortest_path_with_multi_goals::<usize, usize, IGraph, U9>(
            start, &is_goal, &graph,
        );
        assert!(path.is_some());
        let path = path.unwrap();

        let is_checker = get_checker_nodes::<usize, usize, IGraph, U9>(&path, &graph);
        let expected = [false, true, false, true, false, false, false, true, true];
        assert_eq!(is_checker.as_ref(), expected);
    }
}
