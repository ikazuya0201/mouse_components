mod agent;
mod counter;
mod maze;
mod mode;
mod searcher;
mod solver;
mod switch;

use core::marker::PhantomData;
use core::sync::atomic::Ordering;

pub use agent::Agent;
pub use counter::Counter;
pub use maze::{
    CheckerGraph, DirectionInstructor, Graph, GraphConverter, GraphTranslator, ReducedGraph,
    Storable,
};
use mode::{AtomicMode, Mode};
use searcher::Searcher;
pub use solver::Solver;
pub use switch::Switch;

pub struct Operator<Node, SearchNode, Cost, Position, Direction, Maze, IAgent, ISolver, SW, C> {
    maze: Maze,
    agent: IAgent,
    solver: ISolver,
    mode: AtomicMode,
    switch: SW,
    counter: C,
    searcher: Searcher<Node, SearchNode>,
    _node: PhantomData<fn() -> Node>,
    _search_node: PhantomData<fn() -> SearchNode>,
    _cost: PhantomData<fn() -> Cost>,
    _position: PhantomData<fn() -> Position>,
    _direction: PhantomData<fn() -> Direction>,
}

impl<Node, SearchNode, Cost, Position, Direction, Maze, IAgent, ISolver, SW, C>
    Operator<Node, SearchNode, Cost, Position, Direction, Maze, IAgent, ISolver, SW, C>
where
    SearchNode: Copy + Clone,
    Maze: Storable
        + GraphTranslator<Position>
        + DirectionInstructor<SearchNode, Direction>
        + Graph<Node, Cost>
        + Graph<SearchNode, Cost>
        + GraphConverter<Node, SearchNode>,
    IAgent: Agent<Position, Direction>,
    ISolver: Solver<Node, SearchNode, Cost, Maze>,
    SW: Switch,
    C: Counter,
{
    pub fn new(maze: Maze, agent: IAgent, solver: ISolver, switch: SW, counter: C) -> Self {
        let start = solver.start_search_node();
        Self {
            maze: maze,
            agent: agent,
            solver: solver,
            mode: AtomicMode::new(Mode::Idle),
            switch: switch,
            counter: counter,
            searcher: Searcher::new(start),
            _node: PhantomData,
            _search_node: PhantomData,
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
            Idle => (),
            Search => self.searcher.tick(&self.maze, &self.agent),
            Select => return,
            _ => (),
        }
        self.store_if_switch_enabled(Select);
    }

    fn store_if_switch_enabled(&self, mode: Mode) {
        if self.switch.is_enabled() {
            self.mode.store(mode, Ordering::Relaxed);
        }
    }

    pub fn run(&self) {
        use Mode::*;
        loop {
            match self.mode.load(Ordering::Relaxed) {
                Idle => (),
                Search => {
                    if !self.searcher.search(&self.maze, &self.solver) {
                        self.mode.store(FastRun, Ordering::Relaxed);
                    }
                }
                FastRun => self.fast_run(),
                Select => self.mode_select(),
            }
        }
    }

    fn fast_run(&self) {}

    fn mode_select(&self) {
        self.counter.reset();
        let mut mode = Mode::Idle;
        //waiting for switch off
        while self.switch.is_enabled() {}

        while !self.switch.is_enabled() {
            let count = self.counter.count();
            mode = Mode::from(count % Mode::size());
        }
        self.mode.store(mode, Ordering::Relaxed);

        while self.switch.is_enabled() {}
    }
}
