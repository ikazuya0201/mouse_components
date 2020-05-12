use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use super::{Agent, DirectionInstructor, Graph, GraphConverter, ObstacleInterpreter, Solver};

pub struct Searcher<SearchNode> {
    current: Cell<SearchNode>,
    is_updated: AtomicBool,
}

impl<SearchNode> Searcher<SearchNode>
where
    SearchNode: Clone + Copy,
{
    pub fn new(start: SearchNode) -> Self {
        Self {
            current: Cell::new(start),
            is_updated: AtomicBool::new(true),
        }
    }

    pub fn tick<Position, Direction, Maze, IAgent>(&self, maze: &Maze, agent: &IAgent)
    where
        Maze: ObstacleInterpreter<Position> + DirectionInstructor<SearchNode, Direction>,
        IAgent: Agent<Position, Direction>,
    {
        let obstacles = agent.existing_obstacles();
        maze.interpret_obstacles(obstacles);
        if let Some((direction, node)) = maze.instruct(self.current.get()) {
            agent.set_instructed_direction(direction);
            self.current.set(node);
            self.is_updated.store(true, Ordering::Relaxed);
        }
        agent.track_next();
    }

    //return: false if search finished
    pub fn search<Node, Cost, Direction, Maze, ISolver>(
        &self,
        maze: &Maze,
        solver: &ISolver,
    ) -> bool
    where
        Maze: Graph<Node, Cost>
            + Graph<SearchNode, Cost>
            + GraphConverter<Node, SearchNode>
            + DirectionInstructor<SearchNode, Direction>,
        ISolver: Solver<Node, SearchNode, Cost, Maze>,
    {
        if !self
            .is_updated
            .compare_and_swap(true, false, Ordering::Relaxed)
        {
            return true;
        }
        let current = self.current.get();
        if let Some(candidates) = solver.next_node_candidates(current, maze) {
            maze.update_node_candidates(candidates);
            true
        } else {
            false
        }
    }
}
