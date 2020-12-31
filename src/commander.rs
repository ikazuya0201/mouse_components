use core::cell::{Cell, RefCell};
use core::cmp::Reverse;
use core::convert::TryInto;
use core::fmt::Debug;
use core::marker::PhantomData;
use core::ops::Deref;

use generic_array::GenericArray;
use heap::BinaryHeap;
use heapless::{consts::*, Vec};
use num::{Bounded, Saturating};

use crate::operators::{FinishError, RunCommander, SearchCommander};
use crate::utils::{array_length::ArrayLength, itertools::repeat_n};

pub trait GraphConverter<Node> {
    type SearchNode;
    type SearchNodes: IntoIterator<Item = Self::SearchNode>;

    ///checker nodes are search nodes on which agent can check
    ///if walls on the given path exist.
    fn convert_to_checker_nodes<Nodes: Deref<Target = [Node]>>(
        &self,
        path: Nodes,
    ) -> Self::SearchNodes;
}

pub trait Graph<Node> {
    type Cost;
    type Edges: IntoIterator<Item = (Node, Self::Cost)>;

    fn successors(&self, node: &Node) -> Self::Edges;
    fn predecessors(&self, node: &Node) -> Self::Edges;
}

pub trait ObstacleInterpreter<Obstacle> {
    type Error;

    fn interpret_obstacles<Obstacles: IntoIterator<Item = Obstacle>>(
        &self,
        obstacles: Obstacles,
    ) -> Result<(), Self::Error>;
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct CannotCheckError;

pub trait NodeChecker<Node> {
    fn is_available(&self, node: &Node) -> Result<bool, CannotCheckError>;
}

pub trait NodeConverter<Node> {
    type Error;
    type Target;

    fn convert(&self, source: &Node) -> Result<Self::Target, Self::Error>;
}

pub trait RouteNode {
    type Error;
    type Route;

    fn route(&self, to: &Self) -> Result<Self::Route, Self::Error>;
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
enum State {
    Waiting,
    Solving,
}

type CandNum = U4;

pub struct Commander<Node, Nodes, SearchNode, Max, Maze, ConverterType> {
    start: Node,
    goals: Nodes,
    current: Cell<SearchNode>,
    state: Cell<State>,
    candidates: RefCell<Vec<SearchNode, CandNum>>,
    maze: Maze,
    converter: ConverterType,
    _node_max: PhantomData<fn() -> Max>,
}

impl<Node, Nodes, SearchNode, Max, Maze, ConverterType>
    Commander<Node, Nodes, SearchNode, Max, Maze, ConverterType>
{
    pub fn new(
        start: Node,
        goals: Nodes,
        search_start: SearchNode,
        maze: Maze,
        converter: ConverterType,
    ) -> Self {
        Self {
            start,
            goals,
            current: Cell::new(search_start),
            state: Cell::new(State::Solving),
            candidates: RefCell::new(Vec::new()),
            maze,
            converter,
            _node_max: PhantomData,
        }
    }
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum CommanderError {
    WaitingError,
    SolveFinishError,
}

impl TryInto<FinishError> for CommanderError {
    type Error = ();

    fn try_into(self) -> Result<FinishError, Self::Error> {
        match self {
            Self::WaitingError => Err(()),
            Self::SolveFinishError => Ok(FinishError),
        }
    }
}

impl<Node, Nodes, Cost, Max, Maze, ConverterType, Obstacle> SearchCommander<Obstacle>
    for Commander<Node, Nodes, Maze::SearchNode, Max, Maze, ConverterType>
where
    Max: ArrayLength<Node>
        + ArrayLength<Maze::SearchNode>
        + ArrayLength<Cost>
        + ArrayLength<Reverse<Cost>>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<(Maze::SearchNode, Reverse<Cost>)>
        + ArrayLength<Option<Node>>
        + ArrayLength<Option<usize>>
        + typenum::Unsigned,
    Nodes: Deref<Target = [Node]>,
    Node: Ord + Copy + Debug + Into<usize>,
    Maze::SearchNode: Ord + Copy + Debug + Into<usize> + RouteNode,
    Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<Node, Cost = Cost>
        + Graph<<Maze as GraphConverter<Node>>::SearchNode, Cost = Cost>
        + GraphConverter<Node>
        + ObstacleInterpreter<Obstacle>
        + NodeChecker<<Maze as GraphConverter<Node>>::SearchNode>,
    ConverterType: NodeConverter<<Maze as GraphConverter<Node>>::SearchNode>,
    <ConverterType as NodeConverter<Maze::SearchNode>>::Error: core::fmt::Debug,
    <Maze::SearchNode as RouteNode>::Error: core::fmt::Debug,
{
    type Error = CommanderError;
    type Command = (
        <ConverterType as NodeConverter<Maze::SearchNode>>::Target,
        <Maze::SearchNode as RouteNode>::Route,
    );

    //TODO: write test
    fn update_obstacles<Obstacles: IntoIterator<Item = Obstacle>>(&self, obstacles: Obstacles) {
        let _ = self.maze.interpret_obstacles(obstacles);
    }

    //must return the current pose and next kind
    //TODO: write test
    fn next_command(&self) -> Result<Self::Command, Self::Error> {
        match self.state.get() {
            State::Solving => {
                let candidates = self
                    .next_node_candidates(&self.current.get())
                    .ok_or(CommanderError::SolveFinishError)?;
                self.candidates.replace(candidates);
                self.state.set(State::Waiting);
                Err(CommanderError::WaitingError)
            }
            State::Waiting => {
                let mut next = None;
                {
                    let candidates = self.candidates.borrow();
                    if candidates.is_empty() {
                        return Err(CommanderError::WaitingError);
                    }
                    for &node in candidates.iter() {
                        let is_available = self
                            .maze
                            .is_available(&node)
                            .map_err(|_| CommanderError::WaitingError)?;
                        if is_available {
                            next = Some(node);
                            break;
                        }
                    }
                }
                self.candidates.replace(Vec::new());
                self.state.set(State::Solving);
                if let Some(next) = next {
                    let current = self.current.get();
                    let kind = current
                        .route(&next)
                        .unwrap_or_else(|err| unreachable!("This is bug: {:?}", err));
                    let pose = self
                        .converter
                        .convert(&current)
                        .unwrap_or_else(|err| unreachable!("This is bug: {:?}", err));
                    self.current.set(next);
                    Ok((pose, kind))
                } else {
                    Err(CommanderError::SolveFinishError)
                }
            }
        }
    }
}

//TODO: write test
impl<Node, Nodes, Cost, Max, Maze, ConverterType>
    Commander<Node, Nodes, Maze::SearchNode, Max, Maze, ConverterType>
where
    Max: ArrayLength<Node>
        + ArrayLength<Maze::SearchNode>
        + ArrayLength<Cost>
        + ArrayLength<Reverse<Cost>>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<(Maze::SearchNode, Reverse<Cost>)>
        + ArrayLength<Option<Node>>
        + ArrayLength<Option<usize>>
        + typenum::Unsigned,
    Nodes: Deref<Target = [Node]>,
    Node: Ord + Copy + Debug + Into<usize>,
    Maze::SearchNode: Ord + Copy + Debug + Into<usize>,
    Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<Node, Cost = Cost>
        + Graph<<Maze as GraphConverter<Node>>::SearchNode, Cost = Cost>
        + GraphConverter<Node>,
{
    fn next_node_candidates(
        &self,
        current: &Maze::SearchNode,
    ) -> Option<Vec<Maze::SearchNode, U4>> {
        let shortest_path = self.compute_shortest_path()?;
        let checker_nodes = self.maze.convert_to_checker_nodes(shortest_path);

        let candidates = self
            .maze
            .successors(current)
            .into_iter()
            .collect::<Vec<(Maze::SearchNode, Cost), U4>>();

        let mut dists = repeat_n(Cost::max_value(), Max::USIZE).collect::<GenericArray<_, Max>>();
        let mut heap = BinaryHeap::<Maze::SearchNode, Reverse<Cost>, Max>::new();
        for node in checker_nodes {
            heap.push_or_update(node, Reverse(Cost::min_value()))
                .unwrap();
            dists[node.into()] = Cost::min_value();
        }
        while let Some((node, Reverse(cost))) = heap.pop() {
            if candidates.iter().any(|&(cand, _)| cand == node) {
                continue;
            }
            for (next, edge_cost) in self.maze.predecessors(&node) {
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
            .collect::<Vec<(Maze::SearchNode, Cost), U4>>();

        if candidates.is_empty() {
            return None;
        }

        candidates.sort_unstable_by_key(|elem| elem.1);
        Some(candidates.into_iter().map(|(node, _)| node).collect())
    }
}

//TODO: propagate each conversion errors
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RunCommanderError {
    UnreachableError,
    ConversionError,
}

///NOTO: multiple implementations of Converter<_> can exist for different Target,
///but we assume there is only an implementation.
//TODO: write test
impl<Node, Nodes, SearchNode, Max, Maze, ConverterType> RunCommander
    for Commander<Node, Nodes, SearchNode, Max, Maze, ConverterType>
where
    Max: ArrayLength<Node>
        + ArrayLength<Option<Node>>
        + ArrayLength<Option<usize>>
        + ArrayLength<Maze::Cost>
        + ArrayLength<Reverse<Maze::Cost>>
        + ArrayLength<(Node, Reverse<Maze::Cost>)>
        + ArrayLength<(<ConverterType as NodeConverter<Node>>::Target, Node::Route)>
        + typenum::Unsigned,
    Nodes: Deref<Target = [Node]>,
    Node: Ord + Copy + Debug + Into<usize> + RouteNode,
    Maze::Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<Node>,
    ConverterType: NodeConverter<Node>,
{
    type Error = RunCommanderError;
    type Command = (<ConverterType as NodeConverter<Node>>::Target, Node::Route);
    type Commands = Vec<Self::Command, Max>;

    fn compute_commands(&self) -> Result<Self::Commands, Self::Error> {
        let path = self
            .compute_shortest_path()
            .ok_or(RunCommanderError::UnreachableError)?;

        let mut commands = Vec::new();
        if path.len() == 0 {
            return Ok(commands);
        }
        for i in 0..path.len() - 1 {
            commands
                .push((
                    self.converter
                        .convert(&path[i])
                        .map_err(|_| RunCommanderError::ConversionError)?,
                    path[i]
                        .route(&path[i + 1])
                        .map_err(|_| RunCommanderError::ConversionError)?,
                ))
                .unwrap_or_else(|_| unreachable!());
        }
        Ok(commands)
    }
}

impl<Node, Nodes, SearchNode, Max, Maze, ConverterType>
    Commander<Node, Nodes, SearchNode, Max, Maze, ConverterType>
where
    Max: ArrayLength<Node>
        + ArrayLength<Option<Node>>
        + ArrayLength<Option<usize>>
        + ArrayLength<Maze::Cost>
        + ArrayLength<Reverse<Maze::Cost>>
        + ArrayLength<(Node, Reverse<Maze::Cost>)>
        + typenum::Unsigned,
    Nodes: Deref<Target = [Node]>,
    Node: Ord + Copy + Debug + Into<usize>,
    Maze::Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<Node>,
{
    pub fn compute_shortest_path(&self) -> Option<Vec<Node, Max>> {
        let mut dists =
            repeat_n(Maze::Cost::max_value(), Max::USIZE).collect::<GenericArray<_, Max>>();
        dists[self.start.into()] = Maze::Cost::min_value();

        let mut prev = repeat_n(None, Max::USIZE).collect::<GenericArray<Option<Node>, Max>>();

        let mut heap = BinaryHeap::<Node, Reverse<Maze::Cost>, Max>::new();
        heap.push(self.start, Reverse(Maze::Cost::min_value()))
            .unwrap();

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
            for &goal in self.goals.iter() {
                if node == goal {
                    return construct_path(goal, prev);
                }
            }
            for (next, edge_cost) in self.maze.successors(&node) {
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
    use heapless::consts::*;

    use super::{Commander, Graph};

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

    impl Graph<usize> for IGraph {
        type Cost = usize;
        type Edges = Vec<(usize, usize)>;

        fn successors(&self, node: &usize) -> Self::Edges {
            let mut result = Vec::new();
            for i in 0..self.n {
                if let Some(cost) = self.mat[*node][i] {
                    result.push((i, cost));
                }
            }
            result
        }

        fn predecessors(&self, node: &usize) -> Self::Edges {
            let mut result = Vec::new();
            for i in 0..self.n {
                if let Some(cost) = self.mat[i][*node] {
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
        let goals = vec![8usize];
        let n = 9;

        let graph = IGraph::new(n, &edges);

        let solver = Commander::<usize, _, usize, U9, _, ()>::new(start, goals, start, graph, ());

        let path = solver.compute_shortest_path();
        let expected = [0, 1, 3, 5, 7, 8];

        assert!(path.is_some());
        assert_eq!(path.unwrap().as_ref(), expected);
    }
}
