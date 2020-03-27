pub mod agent;
pub mod direction;
pub mod maze;
pub mod solver;

use core::cell::RefCell;
use core::marker::PhantomData;

use heapless::{consts::*, Vec};

use agent::{Agent, Position};
use direction::{AbsoluteDirection, RelativeDirection};
use maze::{Cost, Graph, GraphTranslator, Node};
use solver::Solver;

pub struct Operator<N, C, AD, P, RD, M, A, S>
where
    N: Node,
    C: Cost,
    AD: AbsoluteDirection,
    P: Position,
    RD: RelativeDirection,
    M: Graph<N, C, AD> + GraphTranslator<N, P>,
    A: Agent<P, RD>,
    S: Solver<N, C, AD, M>,
{
    maze: M,
    agent: A,
    solver: S,
    _node: PhantomData<fn() -> N>,
    _cost: PhantomData<fn() -> C>,
    _relative_direction: PhantomData<fn() -> RD>,
    _position: PhantomData<fn() -> P>,
    _absolute_direction: PhantomData<fn() -> AD>,
}

impl<N, C, AD, P, RD, M, A, S> Operator<N, C, AD, P, RD, M, A, S>
where
    N: Node,
    C: Cost,
    AD: AbsoluteDirection,
    P: Position,
    RD: RelativeDirection,
    M: Graph<N, C, AD> + GraphTranslator<N, P>,
    A: Agent<P, RD>,
    S: Solver<N, C, AD, M>,
{
    pub fn new(maze: M, agent: A, solver: S) -> Self {
        Self {
            maze: maze,
            agent: agent,
            solver: solver,
            _node: PhantomData,
            _cost: PhantomData,
            _absolute_direction: PhantomData,
            _position: PhantomData,
            _relative_direction: PhantomData,
        }
    }

    pub fn tick(&self) {
        self.agent.track_next();
    }

    pub fn run(&self) {
        loop {
            while !self.agent.is_route_empty() {}
            let current_node = self.maze.position_to_node(self.agent.position());
            let (route, directions) = self.solver.solve::<U1024>(current_node, &self.maze);
            let route = route
                .into_iter()
                .map(|n| self.maze.node_to_position(n))
                .collect::<Vec<P, U1024>>();
            self.agent.set_next_route(&route);
        }
    }
}
