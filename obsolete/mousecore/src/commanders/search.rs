use core::convert::{Infallible, TryFrom, TryInto};
use core::fmt::Debug;
use core::marker::PhantomData;
use core::ops::Deref;

use heapless::{binary_heap::Min, BinaryHeap, Vec};
use num_traits::{PrimInt, Unsigned};
use serde::{Deserialize, Serialize};
use spin::Mutex;

use super::{
    AsId, AsIndex, CommanderState, CostNode, GeometricGraph, GoalVec, Graph, RouteNode, SsspSolver,
    HEAP_SIZE, NODE_NUMBER_UPPER_BOUND,
};
use crate::operators::{TrackingCommander, TrackingCommanderError};
use crate::{Construct, Deconstruct, Merge};

/// A trait that converts a given path to nodes which are on the path.
pub trait UncheckedNodeFinder<Node> {
    type SearchNode;
    type SearchNodes: IntoIterator<Item = Self::SearchNode>;

    /// Finds search nodes which are on the given path.
    fn find_unchecked_nodes(&self, path: &[Node]) -> Self::SearchNodes;
}

/// A trait that checks whether a given node can be visited or not.
///
/// This returns None when it does not have enough information to check.
pub trait NodeChecker<Node> {
    fn is_available(&self, node: &Node) -> Option<bool>;
}

/// A trait that returns a node determined by itself and a given route.
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

const CANDIDATE_SIZE_UPPER_BOUND: usize = 4;

/// An implementation of [TrackingCommander](crate::operators::TrackingCommander) required by
/// [TrackingOperator](crate::operators::TrackingOperator).
pub struct SearchCommander<Node, RunNode, SearchNode, Route, Position, Maze, Solver> {
    start: RunNode,
    goals: GoalVec<RunNode>,
    initial_route: Route,
    final_route: Route,
    current: Mutex<Node>,
    state: Mutex<State>,
    candidates: Mutex<Vec<SearchNode, CANDIDATE_SIZE_UPPER_BOUND>>,
    maze: Maze,
    solver: Solver,
    _position: PhantomData<fn() -> Position>,
}

impl<Node, RunNode, SearchNode, Route, Position, Maze, Solver>
    SearchCommander<Node, RunNode, SearchNode, Route, Position, Maze, Solver>
where
    RunNode: Clone,
    Position: Into<GoalVec<RunNode>>,
{
    pub fn new(
        start: RunNode,
        goal: Position,
        current: Node,
        initial_route: Route,
        final_route: Route,
        maze: Maze,
        solver: Solver,
    ) -> Self {
        Self {
            start,
            goals: goal.into(),
            initial_route,
            final_route,
            current: Mutex::new(current),
            state: Mutex::new(State::Initial),
            candidates: Mutex::new(Vec::new()),
            maze,
            solver,
            _position: PhantomData,
        }
    }
}

impl<Node, RunNode, SearchNode, Route, Position, Maze, Solver>
    SearchCommander<Node, RunNode, SearchNode, Route, Position, Maze, Solver>
{
    pub fn release(self) -> (Node, Maze, Solver) {
        let Self {
            current,
            maze,
            solver,
            ..
        } = self;
        (current.into_inner(), maze, solver)
    }
}

/// Config for [SearchCommander].
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct SearchCommanderConfig<RunNode, Route, Position> {
    pub start: RunNode,
    pub goal: Position,
    pub initial_route: Route,
    pub final_route: Route,
}

impl<Node, RunNode, SearchNode, Route, Position, Maze, Solver, Config, State, Resource>
    Construct<Config, State, Resource>
    for SearchCommander<Node, RunNode, SearchNode, Route, Position, Maze, Solver>
where
    Maze: Construct<Config, State, Resource>,
    Solver: Construct<Config, State, Resource>,
    Config: AsRef<SearchCommanderConfig<RunNode, Route, Position>>,
    State: AsRef<CommanderState<Node>>,
    Node: Clone,
    RunNode: Clone,
    Route: Clone,
    Position: Clone + Into<GoalVec<RunNode>>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let maze = Maze::construct(config, state, resource);
        let solver = Solver::construct(config, state, resource);
        let config = config.as_ref();
        let state = state.as_ref();
        Self::new(
            config.start.clone(),
            config.goal.clone(),
            state.current_node.clone(),
            config.initial_route.clone(),
            config.final_route.clone(),
            maze,
            solver,
        )
    }
}

impl<Node, RunNode, SearchNode, Route, Position, Maze, Solver, State, Resource>
    Deconstruct<State, Resource>
    for SearchCommander<Node, RunNode, SearchNode, Route, Position, Maze, Solver>
where
    RunNode: Clone,
    Maze: Deconstruct<State, Resource>,
    Solver: Deconstruct<State, Resource>,
    State: From<CommanderState<Node>> + Merge,
    Resource: Merge,
{
    fn deconstruct(self) -> (State, Resource) {
        let (current_node, maze, solver) = self.release();
        let (maze_state, maze_resource) = maze.deconstruct();
        let (solver_state, solver_resource) = solver.deconstruct();
        (
            maze_state
                .merge(solver_state)
                .merge(CommanderState { current_node }.into()),
            maze_resource.merge(solver_resource),
        )
    }
}

impl<Node, RunNode, SearchNode, Route, Position, Maze, Solver> core::fmt::Debug
    for SearchCommander<Node, RunNode, SearchNode, Route, Position, Maze, Solver>
where
    Maze: core::fmt::Debug,
    SearchNode: core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "candidates: {:?}", self.candidates.lock())?;
        writeln!(f, "maze: {:?}", &self.maze)
    }
}

impl<Node, RunNode, Cost, Position, Maze, Solver> TrackingCommander
    for SearchCommander<
        Node,
        RunNode,
        Maze::SearchNode,
        <Maze::SearchNode as RouteNode>::Route,
        Position,
        Maze,
        Solver,
    >
where
    RunNode: PartialEq + Debug + AsIndex + Clone + AsId + From<RunNode::Id>,
    RunNode::Id: Clone,
    Maze::SearchNode: PartialEq
        + AsIndex
        + RouteNode
        + TryFrom<Node>
        + Clone
        + AsId
        + From<<Maze::SearchNode as AsId>::Id>,
    Cost: PrimInt + Unsigned + Debug,
    Maze: GeometricGraph<RunNode, Cost = Cost>
        + Graph<<Maze as UncheckedNodeFinder<RunNode>>::SearchNode, Cost = Cost>
        + UncheckedNodeFinder<RunNode>
        + NodeChecker<<Maze as UncheckedNodeFinder<RunNode>>::SearchNode>,
    Node: From<Maze::SearchNode> + Clone + NextNode<<Maze::SearchNode as RouteNode>::Route>,
    <Maze::SearchNode as RouteNode>::Route: Clone,
    <Maze::SearchNode as RouteNode>::Error: core::fmt::Debug,
    Solver: SsspSolver<RunNode, Maze>,
    Solver::Path: Deref<Target = [RunNode]>,
{
    type Error = Infallible;
    type Command = (Node, <Maze::SearchNode as RouteNode>::Route);

    //must return the current pose and next kind
    //TODO: write test
    fn next_command(&self) -> Result<Self::Command, TrackingCommanderError<Self::Error>> {
        let mut state = self.state.lock();
        match *state {
            State::Initial => {
                let mut current = self.current.lock();
                let mut next = current
                    .next(&self.initial_route)
                    .unwrap_or_else(|_| unreachable!());
                core::mem::swap(&mut *current, &mut next);
                *state = State::Solving;
                Ok((next, self.initial_route.clone()))
            }
            State::Final => Err(TrackingCommanderError::TrackingFinish),
            State::Solving => {
                let mut current = self.current.lock();
                if let Some(candidates) = self.next_node_candidates(
                    &current
                        .clone()
                        .try_into()
                        .unwrap_or_else(|_| unimplemented!()),
                ) {
                    *self.candidates.lock() = candidates;
                    *state = State::Waiting;
                    Err(TrackingCommanderError::Waiting)
                } else {
                    let tmp = current.clone();
                    *current = current
                        .next(&self.final_route)
                        .unwrap_or_else(|_| unreachable!());
                    *state = State::Final;
                    Ok((tmp, self.final_route.clone()))
                }
            }
            State::Waiting => {
                let mut next = None;
                let mut candidates = self.candidates.lock();
                if candidates.is_empty() {
                    return Err(TrackingCommanderError::Waiting);
                }
                for node in candidates.iter() {
                    let is_available = self
                        .maze
                        .is_available(node)
                        .ok_or(TrackingCommanderError::Waiting)?;
                    if is_available {
                        next = Some(node.clone());
                        break;
                    }
                }
                candidates.clear();
                let next = next.unwrap_or_else(|| unreachable!("This is bug."));
                let kind = {
                    <Maze::SearchNode as TryFrom<Node>>::try_from(self.current.lock().clone())
                        .unwrap_or_else(|_| unimplemented!())
                        .route(&next)
                        .unwrap_or_else(|err| unreachable!("This is bug: {:?}", err))
                };
                let mut next = Node::from(next);
                let mut current = self.current.lock();
                core::mem::swap(&mut next, &mut current);
                *state = State::Solving;
                Ok((next, kind))
            }
        }
    }
}

//TODO: write test
impl<Node, RunNode, Cost, Route, Position, Maze, Solver>
    SearchCommander<Node, RunNode, Maze::SearchNode, Route, Position, Maze, Solver>
where
    RunNode: PartialEq + Debug + AsIndex + Clone + AsId + From<RunNode::Id>,
    RunNode::Id: Clone,
    Maze::SearchNode: PartialEq + AsIndex + Clone + AsId + From<<Maze::SearchNode as AsId>::Id>,
    Cost: PrimInt + Unsigned + Debug,
    Maze: GeometricGraph<RunNode, Cost = Cost>
        + Graph<<Maze as UncheckedNodeFinder<RunNode>>::SearchNode, Cost = Cost>
        + UncheckedNodeFinder<RunNode>,
    Solver: SsspSolver<RunNode, Maze>,
    Solver::Path: Deref<Target = [RunNode]>,
{
    fn next_node_candidates(
        &self,
        current: &Maze::SearchNode,
    ) -> Option<Vec<Maze::SearchNode, CANDIDATE_SIZE_UPPER_BOUND>> {
        let shortest_path = self
            .solver
            .solve(&self.start, &self.goals, &self.maze)
            .ok()?;
        let checker_nodes = self.maze.find_unchecked_nodes(&shortest_path);

        let candidates = self
            .maze
            .successors(current)
            .into_iter()
            .collect::<Vec<(Maze::SearchNode, Cost), CANDIDATE_SIZE_UPPER_BOUND>>();

        let mut dists = [Cost::max_value(); NODE_NUMBER_UPPER_BOUND];
        let mut heap =
            BinaryHeap::<CostNode<Cost, <Maze::SearchNode as AsId>::Id>, Min, HEAP_SIZE>::new();
        for node in checker_nodes {
            dists[node.as_index()] = Cost::min_value();
            heap.push(CostNode(Cost::min_value(), node.as_id()))
                .unwrap_or_else(|_| {
                    unreachable!("The length of binary heap should never exceed the upper bound")
                });
        }
        while let Some(CostNode(cost, node)) = heap.pop() {
            let node = Maze::SearchNode::from(node);
            if candidates.iter().any(|(cand, _)| cand == &node) {
                continue;
            }
            for (next, edge_cost) in self.maze.predecessors(&node) {
                let next_cost = cost.saturating_add(edge_cost);
                if dists[next.as_index()] > next_cost {
                    dists[next.as_index()] = next_cost;
                    heap.push(CostNode(next_cost, next.as_id()))
                        .unwrap_or_else(|_| {
                            unreachable!(
                                "The length of binary heap should never exceed the upper bound"
                            )
                        });
                }
            }
        }

        let mut candidates = candidates
            .into_iter()
            .map(|(node, cost)| {
                let index = node.as_index();
                (node, cost.saturating_add(dists[index]))
            })
            .filter(|&(_, cost)| cost < Cost::max_value())
            .collect::<Vec<(Maze::SearchNode, Cost), CANDIDATE_SIZE_UPPER_BOUND>>();

        if candidates.is_empty() {
            return None;
        }

        candidates.sort_unstable_by_key(|elem| elem.1);
        Some(candidates.into_iter().map(|(node, _)| node).collect())
    }
}

impl<Node, RunNode, SearchNode, Route, Position, Maze, Solver>
    SearchCommander<Node, RunNode, SearchNode, Route, Position, Maze, Solver>
where
    Solver: SsspSolver<RunNode, Maze>,
{
    pub fn compute_shortest_path(&self) -> Result<Solver::Path, Solver::Error> {
        self.solver.solve(&self.start, &self.goals, &self.maze)
    }
}
