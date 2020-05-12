use core::cell::Cell;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::administrator::{
    Agent, DirectionInstructor, Graph, GraphConverter, Mode, NotFinishError, ObstacleInterpreter,
    Operator, Solver,
};

pub struct SearchOperator<'a, Node, SearchNode, Cost, Direction, Position, Maze, IAgent, ISolver> {
    current: Cell<SearchNode>,
    is_updated: AtomicBool,
    maze: &'a Maze,
    agent: &'a IAgent,
    solver: &'a ISolver,
    _node: PhantomData<fn() -> Node>,
    _cost: PhantomData<fn() -> Cost>,
    _direction: PhantomData<fn() -> Direction>,
    _position: PhantomData<fn() -> Position>,
}

impl<'a, Node, SearchNode, Cost, Direction, Position, Maze, IAgent, ISolver>
    SearchOperator<'a, Node, SearchNode, Cost, Direction, Position, Maze, IAgent, ISolver>
{
    pub fn new(start: SearchNode, maze: &'a Maze, agent: &'a IAgent, solver: &'a ISolver) -> Self {
        Self {
            current: Cell::new(start),
            is_updated: AtomicBool::new(true),
            maze,
            agent,
            solver,
            _node: PhantomData,
            _cost: PhantomData,
            _direction: PhantomData,
            _position: PhantomData,
        }
    }
}

impl<'a, Node, SearchNode, Cost, Direction, Position, Maze, IAgent, ISolver> Operator
    for SearchOperator<'a, Node, SearchNode, Cost, Direction, Position, Maze, IAgent, ISolver>
where
    SearchNode: Clone + Copy,
    Maze: ObstacleInterpreter<Position>
        + Graph<Node, Cost>
        + Graph<SearchNode, Cost>
        + GraphConverter<Node, SearchNode>
        + DirectionInstructor<SearchNode, Direction>,
    IAgent: Agent<Position, Direction>,
    ISolver: Solver<Node, SearchNode, Cost, Maze>,
{
    fn tick(&self) {
        let obstacles = self.agent.existing_obstacles();
        self.maze.interpret_obstacles(obstacles);
        if let Some((direction, node)) = self.maze.instruct(self.current.get()) {
            self.agent.set_instructed_direction(direction);
            self.current.set(node);
            self.is_updated.store(true, Ordering::Relaxed);
        }
        self.agent.track_next();
    }

    //return: false if search finished
    fn run(&self) -> Result<Mode, NotFinishError> {
        if !self
            .is_updated
            .compare_and_swap(true, false, Ordering::Relaxed)
        {
            return Err(NotFinishError);
        }
        let current = self.current.get();
        if let Some(candidates) = self.solver.next_node_candidates(current, &self.maze) {
            self.maze.update_node_candidates(candidates);
            Ok(Mode::FastRun)
        } else {
            Err(NotFinishError)
        }
    }
}
