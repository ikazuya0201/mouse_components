use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use heapless::{consts::*, Vec};

use super::{Agent, DirectionInstructor, Graph, GraphTranslator, Solver};

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

    pub fn tick<Cost, Position, Direction, M, A>(&self, maze: &M, agent: &A)
    where
        M: Graph<Node, Cost, Direction>
            + GraphTranslator<Node, Position>
            + DirectionInstructor<Node, Direction>,
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

    pub fn search<Cost, Position, Direction, M, A, S>(&self, maze: &M, agent: &A, solver: &S)
    where
        M: Graph<Node, Cost, Direction>
            + GraphTranslator<Node, Position>
            + DirectionInstructor<Node, Direction>,
        A: Agent<Position, Direction>,
        S: Solver<Node, Cost, Direction, M>,
    {
        if !self
            .is_updated
            .compare_and_swap(true, false, Ordering::Relaxed)
        {
            return;
        }
        let current = self.current.get();
        let (route, mapping) = solver.solve::<U1024>(current, &maze);
        maze.set_direction_mapping(current, mapping);
        let route = route
            .into_iter()
            .map(|n| maze.node_to_position(n))
            .collect::<Vec<Position, U1024>>();
        agent.set_next_route(&route);
    }
}
