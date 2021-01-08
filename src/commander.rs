use core::cell::{Cell, RefCell};
use core::cmp::Reverse;
use core::convert::{TryFrom, TryInto};
use core::fmt::Debug;

use generic_array::GenericArray;
use heap::BinaryHeap;
use heapless::{consts::*, Vec};
use num::{Bounded, Saturating};
use typenum::Unsigned;

use crate::operators::{FinishError, RunCommander, SearchCommander};
use crate::utils::{array_length::ArrayLength, forced_vec::ForcedVec, itertools::repeat_n};

pub trait GraphConverter<Node> {
    type SearchNode;
    type SearchNodes: IntoIterator<Item = Self::SearchNode>;

    ///checker nodes are search nodes on which agent can check
    ///if walls on the given path exist.
    fn convert_to_checker_nodes<Nodes: core::ops::Deref<Target = [Node]>>(
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

pub trait RouteNode {
    type Error;
    type Route;

    fn route(&self, to: &Self) -> Result<Self::Route, Self::Error>;
}

pub trait BoundedPathNode {
    type PathUpperBound;
}

pub trait BoundedNode {
    type UpperBound;
}

pub trait NextNode<Route>: Sized {
    type Error;

    fn next(&self, route: &Route) -> Result<Self, Self::Error>;
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
enum State {
    Waiting,
    Solving,
    Initial,
    Final,
}

type CandidateSizeUpperBound = U4;
type GoalSizeUpperBound = U8;

//NOTE: The upper bounds of goal size and candidates size are fixed.
pub struct Commander<Node, RunNode, SearchNode, Route, Maze> {
    start: RunNode,
    goals: Vec<RunNode, GoalSizeUpperBound>,
    initial_route: Route,
    final_route: Route,
    current: RefCell<Node>,
    state: Cell<State>,
    candidates: RefCell<Vec<SearchNode, CandidateSizeUpperBound>>,
    maze: Maze,
}

impl<Node, RunNode, SearchNode, Route, Maze> Commander<Node, RunNode, SearchNode, Route, Maze>
where
    Node: From<RunNode>,
    RunNode: Clone,
{
    pub fn new<RunNodes: IntoIterator<Item = RunNode>>(
        start: RunNode,
        goals: RunNodes,
        initial_route: Route,
        final_route: Route,
        maze: Maze,
    ) -> Self {
        Self {
            start: start.clone(),
            goals: goals.into_iter().collect(),
            initial_route,
            final_route,
            current: RefCell::new(start.into()),
            state: Cell::new(State::Initial),
            candidates: RefCell::new(Vec::new()),
            maze,
        }
    }
}

impl<Node, RunNode, SearchNode, Route, Maze> core::fmt::Debug
    for Commander<Node, RunNode, SearchNode, Route, Maze>
where
    Maze: core::fmt::Debug,
    SearchNode: core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "candidates: {:?}", self.candidates.borrow())?;
        writeln!(f, "maze: {:?}", &self.maze)
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

impl<Node, RunNode, Cost, Maze, Obstacle> SearchCommander<Obstacle>
    for Commander<Node, RunNode, Maze::SearchNode, <Maze::SearchNode as RouteNode>::Route, Maze>
where
    RunNode::UpperBound: ArrayLength<Maze::SearchNode>
        + ArrayLength<Cost>
        + ArrayLength<Reverse<Cost>>
        + ArrayLength<(RunNode, Reverse<Cost>)>
        + ArrayLength<(Maze::SearchNode, Reverse<Cost>)>
        + ArrayLength<Option<RunNode>>
        + ArrayLength<Option<usize>>
        + Unsigned,
    RunNode::PathUpperBound: ArrayLength<RunNode>,
    RunNode: PartialEq + Copy + Debug + Into<usize> + BoundedNode + BoundedPathNode,
    Maze::SearchNode: PartialEq + Copy + Debug + Into<usize> + RouteNode + TryFrom<Node>,
    Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<RunNode, Cost = Cost>
        + Graph<<Maze as GraphConverter<RunNode>>::SearchNode, Cost = Cost>
        + GraphConverter<RunNode>
        + ObstacleInterpreter<Obstacle>
        + NodeChecker<<Maze as GraphConverter<RunNode>>::SearchNode>,
    Node: From<Maze::SearchNode> + Clone + NextNode<<Maze::SearchNode as RouteNode>::Route>,
    <Maze::SearchNode as RouteNode>::Route: Clone,
    <Maze::SearchNode as RouteNode>::Error: core::fmt::Debug,
{
    type Error = CommanderError;
    type Command = (Node, <Maze::SearchNode as RouteNode>::Route);

    //TODO: write test
    fn update_obstacles<Obstacles: IntoIterator<Item = Obstacle>>(&self, obstacles: Obstacles) {
        let _ = self.maze.interpret_obstacles(obstacles);
    }

    //must return the current pose and next kind
    //TODO: write test
    fn next_command(&self) -> Result<Self::Command, Self::Error> {
        match self.state.get() {
            State::Initial => {
                let next = {
                    self.current
                        .borrow()
                        .next(&self.initial_route)
                        .unwrap_or_else(|_| unreachable!())
                };
                let current = self.current.replace(next);
                self.state.set(State::Solving);
                Ok((current, self.initial_route.clone()))
            }
            State::Final => Err(CommanderError::SolveFinishError),
            State::Solving => {
                let candidates = self
                    .next_node_candidates(
                        &self
                            .current
                            .borrow()
                            .clone()
                            .try_into()
                            .unwrap_or_else(|_| unimplemented!()),
                    )
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
                    let kind = {
                        <Maze::SearchNode as TryFrom<Node>>::try_from(self.current.borrow().clone())
                            .unwrap_or_else(|_| unimplemented!())
                            .route(&next)
                            .unwrap_or_else(|err| unreachable!("This is bug: {:?}", err))
                    };
                    let current = self.current.replace(Node::from(next));
                    Ok((current, kind))
                } else {
                    let next = {
                        self.current
                            .borrow()
                            .next(&self.final_route)
                            .unwrap_or_else(|_| unreachable!())
                    };
                    let current = self.current.replace(next);
                    self.state.set(State::Final);
                    Ok((current, self.final_route.clone()))
                }
            }
        }
    }
}

//TODO: write test
impl<Node, RunNode, Cost, Route, Maze> Commander<Node, RunNode, Maze::SearchNode, Route, Maze>
where
    RunNode::UpperBound: ArrayLength<Maze::SearchNode>
        + ArrayLength<Cost>
        + ArrayLength<Reverse<Cost>>
        + ArrayLength<(RunNode, Reverse<Cost>)>
        + ArrayLength<(Maze::SearchNode, Reverse<Cost>)>
        + ArrayLength<Option<RunNode>>
        + ArrayLength<Option<usize>>
        + Unsigned,
    RunNode::PathUpperBound: ArrayLength<RunNode>,
    RunNode: PartialEq + Copy + Debug + Into<usize> + BoundedNode + BoundedPathNode,
    Maze::SearchNode: PartialEq + Copy + Debug + Into<usize>,
    Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<RunNode, Cost = Cost>
        + Graph<<Maze as GraphConverter<RunNode>>::SearchNode, Cost = Cost>
        + GraphConverter<RunNode>,
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

        let mut dists = repeat_n(Cost::max_value(), <RunNode::UpperBound as Unsigned>::USIZE)
            .collect::<GenericArray<_, RunNode::UpperBound>>();
        let mut heap = BinaryHeap::<Maze::SearchNode, Reverse<Cost>, RunNode::UpperBound>::new();
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
impl<Node, RunNode, SearchNode, Route, Maze> RunCommander
    for Commander<Node, RunNode, SearchNode, Route, Maze>
where
    RunNode::UpperBound: ArrayLength<Option<RunNode>>
        + ArrayLength<Option<usize>>
        + ArrayLength<Maze::Cost>
        + ArrayLength<Reverse<Maze::Cost>>
        + ArrayLength<(RunNode, Reverse<Maze::Cost>)>
        + ArrayLength<(Node, RunNode::Route)>
        + Unsigned,
    RunNode::PathUpperBound: ArrayLength<RunNode>,
    RunNode: PartialEq + Copy + Debug + Into<usize> + RouteNode + BoundedNode + BoundedPathNode,
    Node: From<RunNode>,
    Maze::Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<RunNode>,
{
    type Error = RunCommanderError;
    type Command = (Node, RunNode::Route);
    type Commands = Vec<Self::Command, RunNode::UpperBound>;

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
                    Node::from(path[i]),
                    path[i]
                        .route(&path[i + 1])
                        .map_err(|_| RunCommanderError::ConversionError)?,
                ))
                .unwrap_or_else(|_| unreachable!());
        }
        Ok(commands)
    }
}

impl<Node, RunNode, SearchNode, Route, Maze> Commander<Node, RunNode, SearchNode, Route, Maze>
where
    RunNode::UpperBound: ArrayLength<Option<RunNode>>
        + ArrayLength<Option<usize>>
        + ArrayLength<Maze::Cost>
        + ArrayLength<Reverse<Maze::Cost>>
        + ArrayLength<(RunNode, Reverse<Maze::Cost>)>
        + Unsigned,
    RunNode::PathUpperBound: ArrayLength<RunNode>,
    RunNode: PartialEq + Copy + Debug + Into<usize> + BoundedNode + BoundedPathNode,
    Maze::Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<RunNode>,
{
    pub fn compute_shortest_path(&self) -> Option<Vec<RunNode, RunNode::PathUpperBound>> {
        let mut dists = repeat_n(
            Maze::Cost::max_value(),
            <RunNode::UpperBound as Unsigned>::USIZE,
        )
        .collect::<GenericArray<_, RunNode::UpperBound>>();
        dists[self.start.into()] = Maze::Cost::min_value();

        let mut prev = repeat_n(None, <RunNode::UpperBound as Unsigned>::USIZE)
            .collect::<GenericArray<Option<RunNode>, RunNode::UpperBound>>();

        let mut heap = BinaryHeap::<RunNode, Reverse<Maze::Cost>, RunNode::UpperBound>::new();
        heap.push(self.start, Reverse(Maze::Cost::min_value()))
            .unwrap();

        let construct_path = |goal: RunNode, prev: GenericArray<_, _>| {
            let mut current = prev[goal.into()]?;
            let mut path = ForcedVec::new();
            path.push(goal);
            path.push(current);
            while let Some(next) = prev[current.into()] {
                path.push(next);
                current = next;
            }
            let len = path.len();
            //reverse
            for i in 0..len / 2 {
                path.swap(i, len - i - 1);
            }
            Some(path.into())
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
    use alloc::vec::Vec;

    use heapless::consts::*;

    use super::*;

    #[test]
    fn test_compute_shortest_path() {
        struct Node(usize);
        #[derive(Clone, Copy, PartialEq, Debug)]
        struct RunNode(usize);
        struct SearchNode(usize);

        impl From<usize> for RunNode {
            fn from(value: usize) -> Self {
                RunNode(value)
            }
        }

        impl From<RunNode> for usize {
            fn from(value: RunNode) -> Self {
                value.0
            }
        }

        impl From<RunNode> for Node {
            fn from(value: RunNode) -> Self {
                Node(value.0)
            }
        }

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

        impl<T> Graph<T> for IGraph
        where
            usize: From<T>,
            T: Clone + From<usize>,
        {
            type Cost = usize;
            type Edges = Vec<(T, usize)>;

            fn successors(&self, node: &T) -> Self::Edges {
                let node = usize::from(node.clone());
                let mut result = Vec::new();
                for i in 0..self.n {
                    if let Some(cost) = self.mat[node][i] {
                        result.push((T::from(i), cost));
                    }
                }
                result
            }

            fn predecessors(&self, node: &T) -> Self::Edges {
                let node = usize::from(node.clone());
                let mut result = Vec::new();
                for i in 0..self.n {
                    if let Some(cost) = self.mat[i][node] {
                        result.push((T::from(i), cost));
                    }
                }
                result
            }
        }

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
        let start = RunNode(0);
        let goals = vec![RunNode(8usize)];
        let n = 9;

        let graph = IGraph::new(n, &edges);

        impl BoundedNode for RunNode {
            type UpperBound = U10;
        }

        impl BoundedPathNode for RunNode {
            type PathUpperBound = U10;
        }

        let solver = Commander::<Node, RunNode, SearchNode, _, _>::new(start, goals, (), (), graph);

        let path = solver.compute_shortest_path();
        let expected = vec![0, 1, 3, 5, 7, 8]
            .into_iter()
            .map(|e| RunNode(e))
            .collect::<Vec<_>>();

        assert!(path.is_some());
        assert_eq!(path.unwrap(), expected.as_slice());
    }
}
