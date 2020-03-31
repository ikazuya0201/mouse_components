pub mod agent;
pub mod counter;
pub mod maze;
mod mode;
pub mod solver;
pub mod switch;

use core::marker::PhantomData;
use core::sync::atomic::Ordering;

use heapless::{consts::*, Vec};

use agent::Agent;
use counter::Counter;
use maze::{Graph, GraphTranslator, Storable};
use mode::{AtomicMode, Mode};
use solver::Solver;
use switch::Switch;

pub struct Operator<Node, Cost, Position, Direction, M, A, S, SW, C>
where
    M: Storable + Graph<Node, Cost, Direction> + GraphTranslator<Node, Position>,
    A: Agent<Position, Direction>,
    S: Solver<Node, Cost, Direction, M>,
    SW: Switch,
    C: Counter,
{
    maze: M,
    agent: A,
    solver: S,
    mode: AtomicMode,
    switch: SW,
    counter: C,
    _node: PhantomData<fn() -> Node>,
    _cost: PhantomData<fn() -> Cost>,
    _position: PhantomData<fn() -> Position>,
    _direction: PhantomData<fn() -> Direction>,
}

impl<Node, Cost, Position, Direction, M, A, S, SW, C>
    Operator<Node, Cost, Position, Direction, M, A, S, SW, C>
where
    M: Storable + Graph<Node, Cost, Direction> + GraphTranslator<Node, Position>,
    A: Agent<Position, Direction>,
    S: Solver<Node, Cost, Direction, M>,
    SW: Switch,
    C: Counter,
{
    pub fn new(maze: M, agent: A, solver: S, switch: SW, counter: C) -> Self {
        Self {
            maze: maze,
            agent: agent,
            solver: solver,
            mode: AtomicMode::new(Mode::Idle),
            switch: switch,
            counter: counter,
            _node: PhantomData,
            _cost: PhantomData,
            _position: PhantomData,
            _direction: PhantomData,
        }
    }

    //called by periodic interrupt
    pub fn tick(&self) {
        use Mode::*;
        use Ordering::Relaxed;
        match self.mode.load(Relaxed) {
            Idle => self.store_if_switch_enabled(Select),
            Search => self.tick_search(),
            _ => (),
        }
    }

    fn store_if_switch_enabled(&self, mode: Mode) {
        if self.switch.is_enabled() {
            self.mode.store(mode, Ordering::Relaxed);
        }
    }

    fn tick_search(&self) {
        self.agent.track_next();
        let obstacles = self.agent.existing_obstacles::<U10>();
        self.maze.update_obstacles(&obstacles);
        self.store_if_switch_enabled(Mode::Select);
    }

    pub fn run(&self) {
        use Mode::*;
        loop {
            match self.mode.load(Ordering::Relaxed) {
                Idle => (),
                Search => self.search(),
                FastRun => self.fast_run(),
                Select => self.mode_select(),
            }
        }
    }

    fn search(&self) {
        if !self.agent.is_route_empty() {
            return;
        }
        let current_node = self.maze.position_to_node(self.agent.position());
        let (route, mapping) = self.solver.solve::<U1024>(current_node, &self.maze);
        let route = route
            .into_iter()
            .map(|n| self.maze.node_to_position(n))
            .collect::<Vec<Position, U1024>>();
        self.agent.set_next_route(&route);
        self.agent.set_direction_mapping(mapping);
    }

    fn fast_run(&self) {}

    fn mode_select(&self) {
        use num::FromPrimitive;
        self.counter.reset();
        let mut mode = Mode::Idle;
        //waiting for switch off
        while self.switch.is_enabled() {}

        while !self.switch.is_enabled() {
            let count = self.counter.count();
            mode = Mode::from_u8(count % Mode::size()).unwrap();
        }
        self.mode.store(mode, Ordering::Relaxed);
    }
}
