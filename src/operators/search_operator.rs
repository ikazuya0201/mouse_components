use alloc::rc::Rc;
use core::cell::Cell;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::administrator::{NotFinishError, Operator};

pub trait SearchAgent<Pose, Direction> {
    type Obstacle;
    type Obstacles: IntoIterator<Item = Self::Obstacle>;

    fn init(&self, pose: &Pose);
    fn get_existing_obstacles(&self) -> Self::Obstacles;
    fn set_instructed_direction(&self, pose: &Pose, direction: &Direction);
    //This method is called by interrupt.
    fn track_next(&self);
}

pub trait ObstacleInterpreter<Obstacle> {
    fn interpret_obstacles<Obstacles: IntoIterator<Item = Obstacle>>(&self, obstacles: Obstacles);
}

//The trait method could be called by other threads.
pub trait DirectionInstructor<Node> {
    type Direction;

    fn update_node_candidates<Nodes: IntoIterator<Item = Node>>(&self, candidates: Nodes);
    fn instruct(&self, current: &Node) -> Option<(Self::Direction, Node)>;
}

///convert node to pose
pub trait NodeConverter<Node> {
    type Pose;

    fn convert(&self, node: &Node) -> Self::Pose;
}

pub trait SearchSolver<Node, Graph> {
    type Nodes: IntoIterator<Item = Node>;

    //return: (route, last direction candidates sequence)
    fn next_node_candidates(&self, current: &Node, graph: &Graph) -> Option<Self::Nodes>;
}

pub struct SearchOperator<Mode, Node, Obstacle, Maze, Agent, Solver>
where
    Maze: NodeConverter<Node>,
{
    start_pose: Maze::Pose,
    start_node: Node,
    current: Cell<Node>,
    is_updated: AtomicBool,
    maze: Rc<Maze>,
    agent: Rc<Agent>,
    solver: Rc<Solver>,
    next_mode: Mode,
    _obstacle: PhantomData<fn() -> Obstacle>,
}

impl<Mode, Node, Obstacle, Maze, Agent, Solver>
    SearchOperator<Mode, Node, Obstacle, Maze, Agent, Solver>
where
    Node: Clone,
    Maze: NodeConverter<Node>,
{
    pub fn new(
        start_pose: Maze::Pose,
        start_node: Node,
        maze: Rc<Maze>,
        agent: Rc<Agent>,
        solver: Rc<Solver>,
        next_mode: Mode,
    ) -> Self {
        Self {
            start_pose,
            start_node: start_node.clone(),
            current: Cell::new(start_node),
            is_updated: AtomicBool::new(true),
            maze,
            agent,
            solver,
            next_mode,
            _obstacle: PhantomData,
        }
    }
}

impl<Mode, Node, Obstacle, Maze, Agent, Solver> Operator
    for SearchOperator<Mode, Node, Obstacle, Maze, Agent, Solver>
where
    Mode: Copy,
    Node: Copy,
    Maze: ObstacleInterpreter<Obstacle> + DirectionInstructor<Node> + NodeConverter<Node>,
    Agent: SearchAgent<Maze::Pose, Maze::Direction, Obstacle = Obstacle>,
    Solver: SearchSolver<Node, Maze>,
{
    type Mode = Mode;

    fn init(&self) {
        self.current.set(self.start_node);
        self.agent.init(&self.start_pose);
    }

    fn tick(&self) {
        let obstacles = self.agent.get_existing_obstacles();
        self.maze.interpret_obstacles(obstacles);
        let current = self.current.get();
        if let Some((direction, node)) = self.maze.instruct(&current) {
            self.agent
                .set_instructed_direction(&self.maze.convert(&current), &direction);
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
        if let Some(candidates) = self.solver.next_node_candidates(&current, &self.maze) {
            self.maze.update_node_candidates(candidates);
            Err(NotFinishError)
        } else {
            Ok(self.next_mode)
        }
    }
}
