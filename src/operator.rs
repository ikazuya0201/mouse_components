pub mod agent;
pub mod maze;
pub mod solver;

use core::marker::PhantomData;

use heapless::{consts::*, ArrayLength, Vec};

use agent::Agent;
use maze::{Graph, GraphTranslator, Storable};
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
            _node: PhantomData,
            _cost: PhantomData,
            _position: PhantomData,
            _direction: PhantomData,
        }
    }

    pub fn tick(&self) {
        self.agent.track_next();
        let obstacles = self.agent.existing_obstacles::<U10>();
        self.maze.update_obstacles(&obstacles);
    }

    pub fn run(&self) {
        loop {
            while !self.agent.is_route_empty() {}
            let current_node = self.maze.position_to_node(self.agent.position());
            let (route, mapping) = self.solver.solve::<U1024>(current_node, &self.maze);
            let route = route
                .into_iter()
                .map(|n| self.maze.node_to_position(n))
                .collect::<Vec<Position, U1024>>();
            self.agent.set_next_route(&route);
            self.agent.set_direction_mapping(mapping);
        }
    }
}
