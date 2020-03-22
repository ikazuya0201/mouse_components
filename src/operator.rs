pub mod agent;
pub mod maze;
pub mod solver;

use core::marker::PhantomData;

use agent::{Agent, Position};
use maze::{Graph, Node, NodePosition};
use solver::Solver;

pub struct Operator<N, P, M, A, S>
where
    N: Node,
    P: Position,
    M: Graph<N> + NodePosition<N, P>,
    A: Agent<P>,
    S: Solver<N>,
{
    maze: M,
    agent: A,
    solver: S,
    _node: PhantomData<fn() -> N>,
    _position: PhantomData<fn() -> P>,
}

impl<N, P, M, A, S> Operator<N, P, M, A, S>
where
    N: Node,
    P: Position,
    M: Graph<N> + NodePosition<N, P>,
    A: Agent<P>,
    S: Solver<N>,
{
    pub fn new(maze: M, agent: A, solver: S) -> Self {
        Self {
            maze: maze,
            agent: agent,
            solver: solver,
            _node: PhantomData,
            _position: PhantomData,
        }
    }

    pub fn tick(&mut self) {
        self.agent.track();
    }

    pub fn run(&mut self) {
        loop {
            let current_node = self.maze.position_to_node(self.agent.position());
        }
    }
}
