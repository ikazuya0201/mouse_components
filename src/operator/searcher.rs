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
    pub fn search<Cost, Position, Direction, M, A, S, L>(
        &self,
        maze: &M,
        agent: &A,
        solver: &S,
    ) -> bool
    where
        M: DirectionalGraph<Node, Cost, Direction>
            + CheckableGraph<Node, Cost, L>
            + GraphTranslator<Node, Position>
            + DirectionInstructor<Node, Direction>,
        A: Agent<Position, Direction>,
        S: Solver<Node, Cost, Direction, M, L>,
        L: ArrayLength<Node>,
    {
        if !self
            .is_updated
            .compare_and_swap(true, false, Ordering::Relaxed)
        {
            return true;
        }
        let current = self.current.get();
        if let Some((route, mapping)) = solver.solve(current, maze) {
            maze.set_direction_mapping(current, mapping);
            let route = route
                .into_iter()
                .map(|n| maze.node_to_position(n))
                .collect::<Vec<Position, U1024>>();
            agent.set_next_route(&route);
            true
        } else {
            false
        }
    }
}
