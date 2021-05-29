use core::fmt::Debug;
use core::marker::PhantomData;
use core::ops::Deref;
use core::sync::atomic::{AtomicUsize, Ordering};

use serde::{Deserialize, Serialize};

use super::{CommanderState, GoalVec, RouteNode, SsspSolver};
use crate::operators::{TrackingCommander, TrackingCommanderError};
use crate::trajectory_generators::RunState;
use crate::{Construct, Deconstruct, Merge};

/// An implementation of [TrackingCommander](crate::operators::TrackingCommander).
pub struct RunCommander<Node, Position, Maze, Solver>
where
    Solver: SsspSolver<Node, Maze>,
{
    current: Node,
    maze: Maze,
    path: Solver::Path,
    iter: AtomicUsize,
    solver: Solver,
    _position: PhantomData<fn() -> Position>,
}

impl<Node, Position, Maze, Solver> RunCommander<Node, Position, Maze, Solver>
where
    Node: Clone,
    Solver: SsspSolver<Node, Maze>,
    Solver::Error: Debug,
    Solver::Path: Deref<Target = [Node]>,
    Position: Into<GoalVec<Node>>,
{
    pub fn new(start: Node, goal: Position, maze: Maze, solver: Solver) -> Self {
        let path = solver
            .solve(&start, &goal.into(), &maze)
            .unwrap_or_else(|err| unimplemented!("{:?}", err));

        Self {
            current: path.last().unwrap_or_else(|| unimplemented!()).clone(),
            maze,
            path,
            iter: AtomicUsize::new(0),
            solver,
            _position: PhantomData,
        }
    }
}

impl<Node, Position, Maze, Solver> RunCommander<Node, Position, Maze, Solver>
where
    Solver: SsspSolver<Node, Maze>,
{
    pub fn release(self) -> (Node, Maze, Solver) {
        let Self {
            current,
            maze,
            solver,
            ..
        } = self;
        (current, maze, solver)
    }
}

/// Config for [RunCommander].
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct RunCommanderConfig<Position> {
    pub goal: Position,
}

impl<Node, Position, Maze, Solver, Config, State, Resource> Construct<Config, State, Resource>
    for RunCommander<Node, Position, Maze, Solver>
where
    Node: Clone,
    Maze: Construct<Config, State, Resource>,
    Solver: Construct<Config, State, Resource> + SsspSolver<Node, Maze>,
    Solver::Error: Debug,
    Solver::Path: Deref<Target = [Node]>,
    Config: AsRef<RunCommanderConfig<Position>>,
    State: AsRef<CommanderState<Node>>,
    Position: Clone + Into<GoalVec<Node>>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let maze = Maze::construct(config, state, resource);
        let solver = Solver::construct(config, state, resource);
        let config = config.as_ref();
        let state = state.as_ref();
        Self::new(
            state.current_node.clone(),
            config.goal.clone(),
            maze,
            solver,
        )
    }
}

impl<Node, Position, Maze, Solver, State, Resource> Deconstruct<State, Resource>
    for RunCommander<Node, Position, Maze, Solver>
where
    Node: RouteNode,
    Maze: Deconstruct<State, Resource>,
    Solver: Deconstruct<State, Resource> + SsspSolver<Node, Maze>,
    State: Merge + From<CommanderState<Node>>,
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

/// Error on [RunCommander](RunCommander).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RunCommanderError {
    UnreachableError,
    ConversionError,
}

/// A command for fast run.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct RunCommand<Node, Route> {
    pub node: Node,
    pub route: Route,
    pub state: RunState,
}

//TODO: write test
impl<Node, Position, Maze, Solver> TrackingCommander for RunCommander<Node, Position, Maze, Solver>
where
    Node: RouteNode + Clone,
    Solver: SsspSolver<Node, Maze>,
    Solver::Path: Deref<Target = [Node]>,
{
    type Error = RunCommanderError;
    type Command = RunCommand<Node, Node::Route>;

    fn next_command(&self) -> Result<Self::Command, TrackingCommanderError<Self::Error>> {
        let iter = self.iter.load(Ordering::Acquire);
        if iter + 1 >= self.path.len() {
            return Err(TrackingCommanderError::TrackingFinish);
        }
        // NOTE: Ignores cases where self.path.len() <= 5.
        let state = if iter <= 1 {
            RunState::Acceleration
        } else if iter + 2 == self.path.len() {
            RunState::Deceleration2
        } else if iter + 3 == self.path.len() {
            RunState::Deceleration1
        } else {
            RunState::Constant
        };
        self.iter.fetch_add(1, Ordering::AcqRel);
        Ok(RunCommand {
            node: self.path[iter].clone(),
            route: self.path[iter]
                .route(&self.path[iter + 1])
                .map_err(|_| TrackingCommanderError::Other(RunCommanderError::ConversionError))?,
            state,
        })
    }
}

/// A commander for returning to start.
pub struct ReturnCommander<Node, Position, Maze, Solver>(
    RunCommander<Node, Position, Maze, Solver>,
)
where
    Solver: SsspSolver<Node, Maze>;

impl<Node, Position, Maze, Solver> TrackingCommander
    for ReturnCommander<Node, Position, Maze, Solver>
where
    Solver: SsspSolver<Node, Maze>,
    Solver::Path: Deref<Target = [Node]>,
    Node: RouteNode + Clone,
{
    type Error = RunCommanderError;
    type Command = RunCommand<Node, Node::Route>;

    fn next_command(&self) -> Result<Self::Command, TrackingCommanderError<Self::Error>> {
        self.0.next_command()
    }
}

/// Config for [ReturnCommander].
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct ReturnCommanderConfig<Position> {
    pub return_goal: Position,
}

impl<Node, Position, Maze, Solver, Config, State, Resource> Construct<Config, State, Resource>
    for ReturnCommander<Node, Position, Maze, Solver>
where
    Node: Clone,
    Maze: Construct<Config, State, Resource>,
    Solver: Construct<Config, State, Resource> + SsspSolver<Node, Maze>,
    Solver::Error: Debug,
    Solver::Path: Deref<Target = [Node]>,
    Config: AsRef<ReturnCommanderConfig<Position>>,
    State: AsRef<CommanderState<Node>>,
    Position: Clone + Into<GoalVec<Node>>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let maze = Maze::construct(config, state, resource);
        let solver = Solver::construct(config, state, resource);
        let config = config.as_ref();
        let state = state.as_ref();
        Self(RunCommander::new(
            state.current_node.clone(),
            config.return_goal.clone(),
            maze,
            solver,
        ))
    }
}

impl<Node, Position, Maze, Solver, State, Resource> Deconstruct<State, Resource>
    for ReturnCommander<Node, Position, Maze, Solver>
where
    Node: RouteNode,
    Maze: Deconstruct<State, Resource>,
    Solver: Deconstruct<State, Resource> + SsspSolver<Node, Maze>,
    State: Merge + From<CommanderState<Node>>,
    Resource: Merge,
{
    fn deconstruct(self) -> (State, Resource) {
        self.0.deconstruct()
    }
}
