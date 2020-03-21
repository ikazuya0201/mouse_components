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
    pub fn tick(&mut self) {
        self.agent.track();
    }

    pub fn run(&mut self) {
        loop {
            let current_node = self.maze.position_to_node(self.agent.position());
        }
    }
}

pub struct OperatorBuilder<NodeType, PositionType, GraphType, AgentType, SolverType> {
    maze: GraphType,
    agent: AgentType,
    solver: SolverType,
    _node: PhantomData<fn() -> NodeType>,
    _position: PhantomData<fn() -> PositionType>,
}

impl OperatorBuilder<(), (), (), (), ()> {
    pub fn new() -> Self {
        Self {
            maze: (),
            agent: (),
            solver: (),
            _node: PhantomData,
            _position: PhantomData,
        }
    }
}

impl<N, P, M, A, S> OperatorBuilder<N, P, M, A, S>
where
    N: Node,
    P: Position,
    M: Graph<N> + NodePosition<N, P>,
    A: Agent<P>,
    S: Solver<N>,
{
    pub fn build(self) -> Operator<N, P, M, A, S> {
        Operator {
            maze: self.maze,
            agent: self.agent,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
        }
    }
}

impl<N, P, A, S> OperatorBuilder<N, P, (), A, S>
where
    N: Node,
    P: Position,
{
    pub fn maze<M>(self, maze: M) -> OperatorBuilder<N, P, M, A, S>
    where
        M: Graph<N> + NodePosition<N, P>,
    {
        OperatorBuilder {
            maze: maze,
            agent: self.agent,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
        }
    }
}

impl<N, A, S> OperatorBuilder<N, (), (), A, S>
where
    N: Node,
{
    pub fn maze<P, M>(self, maze: M) -> OperatorBuilder<N, P, M, A, S>
    where
        P: Position,
        M: Graph<N> + NodePosition<N, P>,
    {
        OperatorBuilder {
            maze: maze,
            agent: self.agent,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
        }
    }
}

impl<P, A, S> OperatorBuilder<(), P, (), A, S>
where
    P: Position,
{
    pub fn maze<N, M>(self, maze: M) -> OperatorBuilder<N, P, M, A, S>
    where
        N: Node,
        M: Graph<N> + NodePosition<N, P>,
    {
        OperatorBuilder {
            maze: maze,
            agent: self.agent,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
        }
    }
}

impl<A, S> OperatorBuilder<(), (), (), A, S> {
    pub fn maze<N, P, M>(self, maze: M) -> OperatorBuilder<N, P, M, A, S>
    where
        N: Node,
        P: Position,
        M: Graph<N> + NodePosition<N, P>,
    {
        OperatorBuilder {
            maze: maze,
            agent: self.agent,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
        }
    }
}

impl<N, P, M, S> OperatorBuilder<N, P, M, (), S>
where
    P: Position,
{
    pub fn agent<A>(self, agent: A) -> OperatorBuilder<N, P, M, A, S>
    where
        A: Agent<P>,
    {
        OperatorBuilder {
            maze: self.maze,
            agent: agent,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
        }
    }
}

impl<N, M, S> OperatorBuilder<N, (), M, (), S> {
    pub fn agent<P, A>(self, agent: A) -> OperatorBuilder<N, P, M, A, S>
    where
        P: Position,
        A: Agent<P>,
    {
        OperatorBuilder {
            maze: self.maze,
            agent: agent,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
        }
    }
}

impl<N, P, M, A> OperatorBuilder<N, P, M, A, ()>
where
    N: Node,
{
    pub fn solver<S>(self, solver: S) -> OperatorBuilder<N, P, M, A, S>
    where
        S: Solver<N>,
    {
        OperatorBuilder {
            maze: self.maze,
            agent: self.agent,
            solver: solver,
            _node: PhantomData,
            _position: PhantomData,
        }
    }
}

impl<P, M, A> OperatorBuilder<(), P, M, A, ()> {
    pub fn solver<N, S>(self, solver: S) -> OperatorBuilder<N, P, M, A, S>
    where
        N: Node,
        S: Solver<N>,
    {
        OperatorBuilder {
            maze: self.maze,
            agent: self.agent,
            solver: solver,
            _node: PhantomData,
            _position: PhantomData,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::agent::AgentDirection;
    use super::*;
    use heapless::{consts::*, ArrayLength, Vec};

    impl Node for u8 {}
    impl Position for u8 {}
    impl Graph<u8> for u8 {
        fn neighbors<L>(&self, node: u8) -> Vec<u8, L>
        where
            L: ArrayLength<u8>,
        {
            Vec::new()
        }
    }
    impl NodePosition<u8, u8> for u8 {
        fn node_to_position(&self, node: u8) -> u8 {
            0
        }
        fn position_to_node(&self, position: u8) -> u8 {
            0
        }
    }

    impl Agent<u8> for u8 {
        fn wall_exists(&mut self, dir: AgentDirection) -> bool {
            false
        }
        fn rotate(&mut self, dir: AgentDirection) {}
        fn position(&self) -> u8 {
            0
        }
        fn set_position_candidates(&mut self, positions: &[u8]) {}
        fn track(&mut self) {}
    }

    impl Solver<u8> for u8 {
        fn solve<M: Graph<u8>, L: ArrayLength<u8>>(
            start: &[u8],
            goals: &[u8],
            maze: &M,
        ) -> Vec<u8, L> {
            Vec::new()
        }
    }

    #[test]
    fn builder_test() {
        let operator = OperatorBuilder::new().agent(0).maze(0).solver(0).build();
    }
}
