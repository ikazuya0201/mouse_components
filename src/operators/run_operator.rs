use alloc::rc::Rc;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::administrator::{NotFinishError, Operator};

pub trait RunSolver<Graph> {
    type Node;
    type Nodes: IntoIterator<Item = Self::Node>;

    fn compute_shortest_path(&self, graph: &Graph) -> Self::Nodes;
}

pub trait RunAgent<Node> {
    type Error;

    fn init_run<Nodes: IntoIterator<Item = Node>>(&self, path: Nodes);
    fn track_next(&self) -> Result<(), Self::Error>;
}

pub struct RunOperator<Mode, Maze, Agent, Solver> {
    next_mode: Mode,
    is_completed: AtomicBool,
    maze: Rc<Maze>,
    agent: Rc<Agent>,
    solver: Rc<Solver>,
}

impl<Mode, Maze, Agent, Solver> RunOperator<Mode, Maze, Agent, Solver> {
    pub fn new(next_mode: Mode, maze: Rc<Maze>, agent: Rc<Agent>, solver: Rc<Solver>) -> Self {
        Self {
            next_mode,
            is_completed: AtomicBool::new(false),
            maze,
            agent,
            solver,
        }
    }
}

impl<Mode, Maze, Agent, Solver> Operator for RunOperator<Mode, Maze, Agent, Solver>
where
    Mode: Copy,
    Agent: RunAgent<Solver::Node>,
    Solver: RunSolver<Maze>,
{
    type Mode = Mode;

    fn init(&self) {
        let path = self.solver.compute_shortest_path(&self.maze);
        self.agent.init_run(path);
    }

    //TODO: correct agent position by sensor value and wall existence
    fn tick(&self) {
        if self.agent.track_next().is_err() {
            self.is_completed.store(true, Ordering::Relaxed);
        }
    }

    fn run(&self) -> Result<Mode, NotFinishError> {
        if self.is_completed.load(Ordering::Relaxed) {
            Ok(self.next_mode)
        } else {
            Err(NotFinishError)
        }
    }
}
