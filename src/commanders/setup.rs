use core::marker::PhantomData;

use serde::{Deserialize, Serialize};
use spin::Mutex;

use super::{CommanderState, GoalVec, SsspSolver};
use crate::operators::{TrackingCommander, TrackingCommanderError};
use crate::{Construct, Deconstruct, Merge};

/// An implementation of [TrackingCommander](crate::operators::TrackingCommander).
///
/// This produces a setup command.
pub struct SetupCommander<Node, Position, Maze, Solver>
where
    Node: RotationNode,
{
    current: Node,
    maze: Maze,
    command: Mutex<Option<(Node, Node::Kind)>>,
    solver: Solver,
    _position: PhantomData<fn() -> Position>,
}

impl<Node, Position, Maze, Solver> SetupCommander<Node, Position, Maze, Solver>
where
    Node: RotationNode,
    Position: Into<GoalVec<Node>>,
    Solver: SsspSolver<Node, Maze>,
{
    pub fn new(current: Node, goal: Position, maze: Maze, solver: Solver) -> Self {
        let goals = goal.into();
        for (node, kind) in current.rotation_nodes() {
            if solver.solve(&node, &goals, &maze).is_ok() {
                return Self {
                    current: node,
                    maze,
                    command: Mutex::new(Some((current, kind))),
                    solver,
                    _position: PhantomData,
                };
            }
        }
        unreachable!()
    }
}

impl<Node, Position, Maze, Solver> SetupCommander<Node, Position, Maze, Solver>
where
    Node: RotationNode,
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

impl<Node, Position, Maze, Solver, State, Resource> Deconstruct<State, Resource>
    for SetupCommander<Node, Position, Maze, Solver>
where
    Node: RotationNode,
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

/// Config for [ReturnSetupCommander].
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct ReturnSetupCommanderConfig<Position> {
    pub return_goal: Position,
}

/// A commander for setting up return.
pub struct ReturnSetupCommander<Node, Position, Maze, Solver>(
    SetupCommander<Node, Position, Maze, Solver>,
)
where
    Node: RotationNode;

impl<Node, Position, Maze, Solver, Config, State, Resource> Construct<Config, State, Resource>
    for ReturnSetupCommander<Node, Position, Maze, Solver>
where
    Node: RotationNode + Clone,
    Maze: Construct<Config, State, Resource>,
    Solver: Construct<Config, State, Resource> + SsspSolver<Node, Maze>,
    Config: AsRef<ReturnSetupCommanderConfig<Position>>,
    State: AsRef<CommanderState<Node>>,
    Position: Clone + Into<GoalVec<Node>>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let maze = Maze::construct(config, state, resource);
        let solver = Solver::construct(config, state, resource);
        let config = config.as_ref();
        let state = state.as_ref();
        Self(SetupCommander::new(
            state.current_node.clone(),
            config.return_goal.clone(),
            maze,
            solver,
        ))
    }
}

/// Config for [ReturnSetupCommander].
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct RunSetupCommanderConfig<Position> {
    pub goal: Position,
}

/// A commander for setting up fast run.
pub struct RunSetupCommander<Node, Position, Maze, Solver>(
    SetupCommander<Node, Position, Maze, Solver>,
)
where
    Node: RotationNode;

impl<Node, Position, Maze, Solver, Config, State, Resource> Construct<Config, State, Resource>
    for RunSetupCommander<Node, Position, Maze, Solver>
where
    Node: RotationNode + Clone,
    Maze: Construct<Config, State, Resource>,
    Solver: Construct<Config, State, Resource> + SsspSolver<Node, Maze>,
    Config: AsRef<RunSetupCommanderConfig<Position>>,
    State: AsRef<CommanderState<Node>>,
    Position: Clone + Into<GoalVec<Node>>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let maze = Maze::construct(config, state, resource);
        let solver = Solver::construct(config, state, resource);
        let config = config.as_ref();
        let state = state.as_ref();
        Self(SetupCommander::new(
            state.current_node.clone(),
            config.goal.clone(),
            maze,
            solver,
        ))
    }
}

/// Error on [ReturnSetupCommander](ReturnSetupCommander).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SetupCommanderError;

/// A trait that enumerates all nodes which is reachable from the given node, and also enumerates
/// kinds of rotation.
pub trait RotationNode: Sized {
    type Kind;
    type Nodes: IntoIterator<Item = (Self, Self::Kind)>;

    fn rotation_nodes(&self) -> Self::Nodes;
}

//TODO: Write test.
impl<Node, Position, Maze, Solver> TrackingCommander
    for SetupCommander<Node, Position, Maze, Solver>
where
    Node: RotationNode,
{
    type Error = SetupCommanderError;
    type Command = (Node, Node::Kind);

    fn next_command(&self) -> Result<Self::Command, TrackingCommanderError<Self::Error>> {
        if let Some(command) = self.command.lock().take() {
            Ok(command)
        } else {
            Err(TrackingCommanderError::TrackingFinish)
        }
    }
}

macro_rules! impl_all_for_setup_commander {
    ($name: ident) => {
        impl<Node, Position, Maze, Solver, State, Resource> Deconstruct<State, Resource>
            for $name<Node, Position, Maze, Solver>
        where
            Node: RotationNode,
            Maze: Deconstruct<State, Resource>,
            Solver: Deconstruct<State, Resource>,
            State: From<CommanderState<Node>> + Merge,
            Resource: Merge,
        {
            fn deconstruct(self) -> (State, Resource) {
                self.0.deconstruct()
            }
        }

        impl<Node, Position, Maze, Solver> TrackingCommander for $name<Node, Position, Maze, Solver>
        where
            Node: RotationNode,
        {
            type Error = SetupCommanderError;
            type Command = (Node, Node::Kind);

            fn next_command(&self) -> Result<Self::Command, TrackingCommanderError<Self::Error>> {
                self.0.next_command()
            }
        }
    };
}

impl_all_for_setup_commander!(ReturnSetupCommander);
impl_all_for_setup_commander!(RunSetupCommander);
