use alloc::rc::Rc;
use core::cell::Cell;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::administrator::{
    Agent, DirectionInstructor, FastRun, Graph, GraphConverter, Mode, NodeConverter,
    NotFinishError, ObstacleInterpreter, Operator, Search, Solver,
};

pub struct SearchOperator<Node, SearchNode, Cost, Direction, Obstacle, Pose, Maze, IAgent, ISolver>
{
    start_pose: Pose,
    start_node: SearchNode,
    current: Cell<SearchNode>,
    is_updated: AtomicBool,
    maze: Rc<Maze>,
    agent: Rc<IAgent>,
    solver: Rc<ISolver>,
    _node: PhantomData<fn() -> Node>,
    _cost: PhantomData<fn() -> Cost>,
    _direction: PhantomData<fn() -> Direction>,
    _obstacle: PhantomData<fn() -> Obstacle>,
    _pose: PhantomData<fn() -> Pose>,
}

impl<Node, SearchNode, Cost, Direction, Obstacle, Pose, Maze, IAgent, ISolver>
    SearchOperator<Node, SearchNode, Cost, Direction, Obstacle, Pose, Maze, IAgent, ISolver>
where
    SearchNode: Copy,
{
    pub fn new(
        start_pose: Pose,
        start_node: SearchNode,
        maze: Rc<Maze>,
        agent: Rc<IAgent>,
        solver: Rc<ISolver>,
    ) -> Self {
        Self {
            start_pose,
            start_node,
            current: Cell::new(start_node),
            is_updated: AtomicBool::new(true),
            maze,
            agent,
            solver,
            _node: PhantomData,
            _cost: PhantomData,
            _direction: PhantomData,
            _obstacle: PhantomData,
            _pose: PhantomData,
        }
    }
}

impl<Node, SearchNode, Cost, Direction, Obstacle, Pose, Maze, IAgent, ISolver> Operator<Search>
    for SearchOperator<Node, SearchNode, Cost, Direction, Obstacle, Pose, Maze, IAgent, ISolver>
where
    SearchNode: Copy,
    Pose: Copy,
    Maze: ObstacleInterpreter<Obstacle>
        + Graph<Node, Cost>
        + Graph<SearchNode, Cost>
        + GraphConverter<Node, SearchNode>
        + DirectionInstructor<SearchNode, Direction>
        + NodeConverter<SearchNode, Pose>,
    IAgent: Agent<Obstacle, Pose, Direction>,
    ISolver: Solver<Node, SearchNode, Cost, Maze>,
{
    fn init(&self) {
        self.current.set(self.start_node);
        self.agent.init(self.start_pose);
    }

    fn tick(&self) {
        let obstacles = self.agent.get_existing_obstacles();
        self.maze.interpret_obstacles(obstacles);
        let current = self.current.get();
        if let Some((direction, node)) = self.maze.instruct(current) {
            self.agent
                .set_instructed_direction(self.maze.convert(current), direction);
            self.current.set(node);
            self.is_updated.store(true, Ordering::Relaxed);
        }
        self.agent.track_next();
    }

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
            Err(NotFinishError)
        } else {
            Ok(Mode::FastRun(FastRun))
        }
    }
}
