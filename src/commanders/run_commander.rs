use core::fmt::Debug;
use core::sync::atomic::{AtomicUsize, Ordering};

use heapless::Vec;
use num_traits::{PrimInt, Unsigned};
use serde::{Deserialize, Serialize};

use super::{
    compute_shortest_path, AsIndex, CommanderState, Distance, Graph, RouteNode,
    GOAL_SIZE_UPPER_BOUND, PATH_UPPER_BOUND,
};
use crate::operators::{TrackingCommander, TrackingCommanderError};
use crate::trajectory_generators::RunState;
use crate::{Construct, Deconstruct, Merge};

/// An implementation of [TrackingCommander](crate::operators::TrackingCommander).
pub struct RunCommander<Node, Maze>
where
    Node: RouteNode,
{
    current: Node,
    maze: Maze,
    path: Vec<Node, PATH_UPPER_BOUND>,
    iter: AtomicUsize,
}

impl<Node, Maze> RunCommander<Node, Maze>
where
    Node: PartialEq + Clone + Debug + AsIndex + RouteNode + Distance<Output = Maze::Cost>,
    Maze::Cost: PrimInt + Unsigned,
    Maze: Graph<Node>,
{
    pub fn new(start: Node, goals: &[Node], maze: Maze) -> Self {
        let path = compute_shortest_path(&start, &goals, &maze).unwrap_or_else(|| unimplemented!());

        Self {
            current: path.last().unwrap_or_else(|| unimplemented!()).clone(),
            maze,
            path,
            iter: AtomicUsize::new(0),
        }
    }
}

impl<Node, Maze> RunCommander<Node, Maze>
where
    Node: RouteNode,
{
    pub fn release(self) -> (Node, Maze) {
        let Self { current, maze, .. } = self;
        (current, maze)
    }
}

/// Config for [RunCommander].
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct RunCommanderConfig<Node> {
    pub goals: Vec<Node, GOAL_SIZE_UPPER_BOUND>,
}

impl<Node, Maze, Config, State, Resource> Construct<Config, State, Resource>
    for RunCommander<Node, Maze>
where
    Node: PartialEq + Clone + Debug + AsIndex + RouteNode + Distance<Output = Maze::Cost>,
    Maze: Construct<Config, State, Resource> + Graph<Node>,
    Maze::Cost: PrimInt + Unsigned,
    Config: AsRef<RunCommanderConfig<Node>>,
    State: AsRef<CommanderState<Node>>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let maze = Maze::construct(config, state, resource);
        let config = config.as_ref();
        let state = state.as_ref();
        Self::new(state.current_node.clone(), &config.goals, maze)
    }
}

impl<Node, Maze, State, Resource> Deconstruct<State, Resource> for RunCommander<Node, Maze>
where
    Node: RouteNode,
    Maze: Deconstruct<State, Resource>,
    State: Merge + From<CommanderState<Node>>,
{
    fn deconstruct(self) -> (State, Resource) {
        let (current_node, maze) = self.release();
        let (state, resource) = maze.deconstruct();
        (
            state.merge(CommanderState { current_node }.into()),
            resource,
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
impl<Node, Maze> TrackingCommander for RunCommander<Node, Maze>
where
    Node: RouteNode + Clone,
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
pub struct ReturnCommander<Node, Maze>(RunCommander<Node, Maze>)
where
    Node: RouteNode;

impl<Node, Maze> TrackingCommander for ReturnCommander<Node, Maze>
where
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
pub struct ReturnCommanderConfig<Node> {
    pub return_goal: Node,
}

impl<Node, Maze, Config, State, Resource> Construct<Config, State, Resource>
    for ReturnCommander<Node, Maze>
where
    Node: PartialEq + Clone + Debug + AsIndex + RouteNode + Distance<Output = Maze::Cost>,
    Maze: Construct<Config, State, Resource> + Graph<Node>,
    Maze::Cost: PrimInt + Unsigned,
    Config: AsRef<ReturnCommanderConfig<Node>>,
    State: AsRef<CommanderState<Node>>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let maze = Maze::construct(config, state, resource);
        let config = config.as_ref();
        let state = state.as_ref();
        Self(RunCommander::new(
            state.current_node.clone(),
            &[config.return_goal.clone()],
            maze,
        ))
    }
}

impl<Node, Maze, State, Resource> Deconstruct<State, Resource> for ReturnCommander<Node, Maze>
where
    Node: RouteNode,
    Maze: Deconstruct<State, Resource>,
    State: Merge + From<CommanderState<Node>>,
{
    fn deconstruct(self) -> (State, Resource) {
        self.0.deconstruct()
    }
}
