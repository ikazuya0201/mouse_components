use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use heapless::{consts::*, ArrayLength, Vec};

use super::{
    Agent, CheckableGraph, DirectionInstructor, DirectionalGraph, GraphTranslator, Solver,
};

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

    pub fn tick<Position, Direction, M, A>(&self, maze: &M, agent: &A)
    where
        M: GraphTranslator<Node, Position> + DirectionInstructor<Node, Direction>,
        A: Agent<Position, Direction>,
    {
        let obstacles = agent.existing_obstacles::<U10>();
        maze.update_obstacles(&obstacles);
        if let Some((direction, node)) = maze.instruct_direction() {
            agent.set_instructed_direction(direction);
            self.current.set(node);
            self.is_updated.store(true, Ordering::Relaxed);
        }
        agent.track_next();
    }

    //return: false if search finished
    pub fn search<Cost, Position, Direction, M, A, S>(
        &self,
        maze: &M,
        agent: &A,
        solver: &S,
    ) -> bool
    where
        M: DirectionalGraph<Node, Cost, Direction>
            + CheckableGraph<Node, Cost>
            + GraphTranslator<Node, Position>
            + DirectionInstructor<Node, Direction>,
        A: Agent<Position, Direction>,
        S: Solver<Node, Cost, Direction, M>,
    {
        if !self
            .is_updated
            .compare_and_swap(true, false, Ordering::Relaxed)
        {
            return true;
        }
        let current = self.current.get();
        if let Some(path) = solver.next_path(current, maze) {
            let path = path.into_iter().map(|n| maze.node_to_position(n));
            agent.set_next_path(path);
            maze.update_direction_candidates(
                solver.last_node().unwrap(),
                solver.next_direction_candidates(maze).unwrap(),
            );
            true
        } else {
            false
        }
    }
}
