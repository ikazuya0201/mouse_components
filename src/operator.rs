pub mod agent;
pub mod maze;
pub mod solver;
pub mod trajectory_generator;

use core::marker::PhantomData;

use agent::{Agent, Position};
use maze::{Maze, Node, NodePosition};
use solver::Solver;
use trajectory_generator::{Target, TrajectoryGenerator};

pub struct Operator<N, P, T, M, A, TG, S>
where
    N: Node,
    P: Position,
    T: Target,
    M: Maze<N> + NodePosition<N, P>,
    A: Agent<P, T>,
    TG: TrajectoryGenerator<P, T>,
    S: Solver<N>,
{
    maze: M,
    agent: A,
    trackgen: TG,
    solver: S,
    _node: PhantomData<fn() -> N>,
    _position: PhantomData<fn() -> P>,
    _target: PhantomData<fn() -> T>,
}

impl<N, P, T, M, A, TG, S> Operator<N, P, T, M, A, TG, S>
where
    N: Node,
    P: Position,
    T: Target,
    M: Maze<N> + NodePosition<N, P>,
    A: Agent<P, T>,
    TG: TrajectoryGenerator<P, T>,
    S: Solver<N>,
{
    pub fn tick(&mut self) {
        self.agent.track();
    }

    pub fn run(&mut self) {
        loop {}
    }
}

pub struct OperatorBuilder<
    NodeType,
    PositionType,
    TargetType,
    MazeType,
    AgentType,
    TrackGenType,
    SolverType,
> {
    maze: MazeType,
    agent: AgentType,
    trackgen: TrackGenType,
    solver: SolverType,
    _node: PhantomData<fn() -> NodeType>,
    _position: PhantomData<fn() -> PositionType>,
    _target: PhantomData<fn() -> TargetType>,
}

impl OperatorBuilder<(), (), (), (), (), (), ()> {
    pub fn new() -> Self {
        Self {
            maze: (),
            agent: (),
            trackgen: (),
            solver: (),
            _node: PhantomData,
            _position: PhantomData,
            _target: PhantomData,
        }
    }
}

impl<N, P, T, M, A, TG, S> OperatorBuilder<N, P, T, M, A, TG, S>
where
    N: Node,
    P: Position,
    T: Target,
    M: Maze<N> + NodePosition<N, P>,
    A: Agent<P, T>,
    TG: TrajectoryGenerator<P, T>,
    S: Solver<N>,
{
    pub fn build(self) -> Operator<N, P, T, M, A, TG, S> {
        Operator {
            maze: self.maze,
            agent: self.agent,
            trackgen: self.trackgen,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
            _target: PhantomData,
        }
    }
}

impl<T, A, TG, S> OperatorBuilder<(), (), T, (), A, TG, S> {
    pub fn maze<N, P, M>(self, maze: M) -> OperatorBuilder<N, P, T, M, A, TG, S>
    where
        N: Node,
        P: Position,
        M: Maze<N> + NodePosition<N, P>,
    {
        OperatorBuilder {
            maze: maze,
            agent: self.agent,
            trackgen: self.trackgen,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
            _target: PhantomData,
        }
    }
}

impl<P, T, A, TG, S> OperatorBuilder<(), P, T, (), A, TG, S>
where
    P: Position,
{
    pub fn maze<N, M>(self, maze: M) -> OperatorBuilder<N, P, T, M, A, TG, S>
    where
        N: Node,
        M: Maze<N> + NodePosition<N, P>,
    {
        OperatorBuilder {
            maze: maze,
            agent: self.agent,
            trackgen: self.trackgen,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
            _target: PhantomData,
        }
    }
}

impl<N, P, T, M, TG, S> OperatorBuilder<N, P, T, M, (), TG, S>
where
    P: Position,
    T: Target,
{
    pub fn agent<A>(self, agent: A) -> OperatorBuilder<N, P, T, M, A, TG, S>
    where
        A: Agent<P, T>,
    {
        OperatorBuilder {
            maze: self.maze,
            agent: agent,
            trackgen: self.trackgen,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
            _target: PhantomData,
        }
    }
}

impl<N, P, M, TG, S> OperatorBuilder<N, P, (), M, (), TG, S>
where
    P: Position,
{
    pub fn agent<T, A>(self, agent: A) -> OperatorBuilder<N, P, T, M, A, TG, S>
    where
        T: Target,
        A: Agent<P, T>,
    {
        OperatorBuilder {
            maze: self.maze,
            agent: agent,
            trackgen: self.trackgen,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
            _target: PhantomData,
        }
    }
}

impl<N, T, M, TG, S> OperatorBuilder<N, (), T, M, (), TG, S>
where
    T: Target,
{
    pub fn agent<P, A>(self, agent: A) -> OperatorBuilder<N, P, T, M, A, TG, S>
    where
        P: Position,
        A: Agent<P, T>,
    {
        OperatorBuilder {
            maze: self.maze,
            agent: agent,
            trackgen: self.trackgen,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
            _target: PhantomData,
        }
    }
}

impl<N, M, TG, S> OperatorBuilder<N, (), (), M, (), TG, S> {
    pub fn agent<P, T, A>(self, agent: A) -> OperatorBuilder<N, P, T, M, A, TG, S>
    where
        P: Position,
        T: Target,
        A: Agent<P, T>,
    {
        OperatorBuilder {
            maze: self.maze,
            agent: agent,
            trackgen: self.trackgen,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
            _target: PhantomData,
        }
    }
}

impl<N, P, T, M, A, S> OperatorBuilder<N, P, T, M, A, (), S>
where
    P: Position,
    T: Target,
{
    pub fn trajectory_generator<TG>(self, trackgen: TG) -> OperatorBuilder<N, P, T, M, A, TG, S>
    where
        TG: TrajectoryGenerator<P, T>,
    {
        OperatorBuilder {
            maze: self.maze,
            agent: self.agent,
            trackgen: trackgen,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
            _target: PhantomData,
        }
    }
}

impl<N, T, M, A, S> OperatorBuilder<N, (), T, M, A, (), S>
where
    T: Target,
{
    pub fn trajectory_generator<P, TG>(self, trackgen: TG) -> OperatorBuilder<N, P, T, M, A, TG, S>
    where
        P: Position,
        TG: TrajectoryGenerator<P, T>,
    {
        OperatorBuilder {
            maze: self.maze,
            agent: self.agent,
            trackgen: trackgen,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
            _target: PhantomData,
        }
    }
}

impl<N, P, M, A, S> OperatorBuilder<N, P, (), M, A, (), S>
where
    P: Position,
{
    pub fn trajectory_generator<T, TG>(self, trackgen: TG) -> OperatorBuilder<N, P, T, M, A, TG, S>
    where
        T: Target,
        TG: TrajectoryGenerator<P, T>,
    {
        OperatorBuilder {
            maze: self.maze,
            agent: self.agent,
            trackgen: trackgen,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
            _target: PhantomData,
        }
    }
}

impl<N, M, A, S> OperatorBuilder<N, (), (), M, A, (), S> {
    pub fn trajectory_generator<P, T, TG>(
        self,
        trackgen: TG,
    ) -> OperatorBuilder<N, P, T, M, A, TG, S>
    where
        P: Position,
        T: Target,
        TG: TrajectoryGenerator<P, T>,
    {
        OperatorBuilder {
            maze: self.maze,
            agent: self.agent,
            trackgen: trackgen,
            solver: self.solver,
            _node: PhantomData,
            _position: PhantomData,
            _target: PhantomData,
        }
    }
}

impl<N, P, T, M, A, TG> OperatorBuilder<N, P, T, M, A, TG, ()>
where
    N: Node,
{
    pub fn solver<S>(self, solver: S) -> OperatorBuilder<N, P, T, M, A, TG, S>
    where
        S: Solver<N>,
    {
        OperatorBuilder {
            maze: self.maze,
            agent: self.agent,
            trackgen: self.trackgen,
            solver: solver,
            _node: PhantomData,
            _position: PhantomData,
            _target: PhantomData,
        }
    }
}

impl<P, T, M, A, TG> OperatorBuilder<(), P, T, M, A, TG, ()> {
    pub fn solver<N, S>(self, solver: S) -> OperatorBuilder<N, P, T, M, A, TG, S>
    where
        N: Node,
        S: Solver<N>,
    {
        OperatorBuilder {
            maze: self.maze,
            agent: self.agent,
            trackgen: self.trackgen,
            solver: solver,
            _node: PhantomData,
            _position: PhantomData,
            _target: PhantomData,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::agent::AgentDirection;
    use super::trajectory_generator::Target;
    use super::*;
    use heapless::{consts::*, ArrayLength, Vec};

    impl Node for u8 {}
    impl Position for u8 {}
    impl Maze<u8> for u8 {
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
    impl Target for u8 {}

    impl Agent<u8, u8> for u8 {
        fn wall_exists(&mut self, dir: AgentDirection) -> bool {
            false
        }
        fn rotate(&mut self, dir: AgentDirection) {}
        fn position(&self) -> u8 {
            0
        }
        fn set_trajectory(&mut self, trajectory: &[u8]) {}
        fn track(&mut self) {}
    }

    impl TrajectoryGenerator<u8, u8> for u8 {
        fn generate<L: ArrayLength<u8>>(&self, positions: &[u8]) -> Vec<u8, L> {
            Vec::new()
        }
    }

    impl Solver<u8> for u8 {
        fn solve<M: Maze<u8>, L: ArrayLength<u8>>(
            start: &[u8],
            goals: &[u8],
            maze: &M,
        ) -> Vec<u8, L> {
            Vec::new()
        }
    }

    #[test]
    fn builder_test() {
        let operator = OperatorBuilder::new()
            .agent(0)
            .maze(0)
            .trajectory_generator(0)
            .solver(0)
            .build();
    }
}
