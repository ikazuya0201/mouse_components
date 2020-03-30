pub mod agent;
pub mod maze;
mod mode;
pub mod solver;

use core::marker::PhantomData;
use core::sync::atomic::Ordering;

use heapless::{consts::*, Vec};

use agent::Agent;
use maze::{Graph, GraphTranslator, Storable};
use mode::{AtomicMode, Mode};
use solver::Solver;

pub struct Operator<Node, Cost, Position, Direction, M, A, S>
where
    M: Storable + Graph<Node, Cost, Direction> + GraphTranslator<Node, Position>,
    A: Agent<Position, Direction>,
    S: Solver<Node, Cost, Direction, M>,
{
    maze: M,
    agent: A,
    solver: S,
    mode: AtomicMode,
    _node: PhantomData<fn() -> Node>,
    _cost: PhantomData<fn() -> Cost>,
    _position: PhantomData<fn() -> Position>,
    _direction: PhantomData<fn() -> Direction>,
}

impl<Node, Cost, Position, Direction, M, A, S> Operator<Node, Cost, Position, Direction, M, A, S>
where
    M: Storable + Graph<Node, Cost, Direction> + GraphTranslator<Node, Position>,
    A: Agent<Position, Direction>,
    S: Solver<Node, Cost, Direction, M>,
{
    pub fn new(maze: M, agent: A, solver: S) -> Self {
        Self {
            maze: maze,
            agent: agent,
            solver: solver,
            mode: AtomicMode::new(Mode::Idle),
            _node: PhantomData,
            _cost: PhantomData,
            _position: PhantomData,
            _direction: PhantomData,
        }
    }

    //called by periodic interrupt
    pub fn tick(&self) {
        self.agent.track_next();
        let obstacles = self.agent.existing_obstacles::<U10>();
        self.maze.update_obstacles(&obstacles);
    }

    pub fn run(&self) {
        use Mode::*;
        loop {
            match self.mode.load(Ordering::Relaxed) {
                Idle => (),
                Search => self.search(),
                FastRun => self.fast_run(),
                Select => self.mode_select(),
            }
        }
    }

    fn search(&self) {
        if !self.agent.is_route_empty() {
            return;
        }
        let current_node = self.maze.position_to_node(self.agent.position());
        let (route, mapping) = self.solver.solve::<U1024>(current_node, &self.maze);
        let route = route
            .into_iter()
            .map(|n| self.maze.node_to_position(n))
            .collect::<Vec<Position, U1024>>();
        self.agent.set_next_route(&route);
        self.agent.set_direction_mapping(mapping);
    }

    fn fast_run(&self) {}

    fn mode_select(&self) {}
}
