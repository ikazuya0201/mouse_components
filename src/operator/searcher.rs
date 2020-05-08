use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use super::{Agent, CheckerGraph, GraphTranslator, PatternInstructor, ReducedGraph, Solver};

pub struct Searcher<Node> {
    current: Cell<Node>,
    is_updated: AtomicBool,
}

impl<Node> Searcher<Node>
where
    Node: Clone + Copy,
{
    pub fn new(start: Node) -> Self {
        Self {
            current: Cell::new(start),
            is_updated: AtomicBool::new(true),
        }
    }

    pub fn tick<Position, Direction, Pattern, Maze, IAgent>(&self, maze: &Maze, agent: &IAgent)
    where
        Maze:
            GraphTranslator<Node, Position, Pattern> + PatternInstructor<Node, Direction, Pattern>,
        IAgent: Agent<Position, Pattern>,
    {
        let obstacles = agent.existing_obstacles();
        maze.update_obstacles(obstacles);
        if let Some((pattern, node)) = maze.instruct() {
            agent.set_instructed_pattern(pattern);
            self.current.set(node);
            self.is_updated.store(true, Ordering::Relaxed);
        }
        agent.track_next();
    }

    //return: false if search finished
    pub fn search<Cost, Position, Direction, Pattern, Maze, IAgent, ISolver>(
        &self,
        maze: &Maze,
        agent: &IAgent,
        solver: &ISolver,
    ) -> bool
    where
        Maze: CheckerGraph<Node>
            + ReducedGraph<Node, Cost>
            + GraphTranslator<Node, Position, Pattern>
            + PatternInstructor<Node, Direction, Pattern>,
        IAgent: Agent<Position, Pattern>,
        ISolver: Solver<Node, Cost>,
    {
        if !self
            .is_updated
            .compare_and_swap(true, false, Ordering::Relaxed)
        {
            return true;
        }
        let current = self.current.get();
        if let Some(candidates) = solver.next_node_candidates(current, maze) {
            maze.update_node_candidates(current, candidates);
            true
        } else {
            false
        }
    }
}
