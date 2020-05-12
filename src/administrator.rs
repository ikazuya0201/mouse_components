mod agent;
mod counter;
mod maze;
mod mode;
mod operator;
mod searcher;
mod solver;
mod switch;

use core::sync::atomic::Ordering;

pub use agent::Agent;
pub use counter::Counter;
pub use maze::{DirectionInstructor, Graph, GraphConverter, ObstacleInterpreter};
use mode::{AtomicMode, Mode};
pub use operator::Operator;
use searcher::Searcher;
pub use solver::Solver;
pub use switch::Switch;

pub struct Administrator<SearchNode, Maze, IAgent, ISolver, SW, C> {
    maze: Maze,
    agent: IAgent,
    solver: ISolver,
    mode: AtomicMode,
    switch: SW,
    counter: C,
    searcher: Searcher<SearchNode>,
}

impl<SearchNode, Maze, IAgent, ISolver, SW, C>
    Administrator<SearchNode, Maze, IAgent, ISolver, SW, C>
where
    SearchNode: Copy + Clone,
    SW: Switch,
    C: Counter,
{
    pub fn new<Node, Cost, Direction, Position>(
        maze: Maze,
        agent: IAgent,
        solver: ISolver,
        switch: SW,
        counter: C,
    ) -> Self
    where
        ISolver: Solver<Node, SearchNode, Cost, Maze>,
    {
        let start = solver.start_search_node();
        Self {
            maze: maze,
            agent: agent,
            solver: solver,
            mode: AtomicMode::new(Mode::Idle),
            switch: switch,
            counter: counter,
            searcher: Searcher::new(start),
        }
    }

    //called by periodic interrupt
    pub fn tick<Node, Cost, Direction, Position>(&self)
    where
        Maze: ObstacleInterpreter<Position> + DirectionInstructor<SearchNode, Direction>,
        IAgent: Agent<Position, Direction>,
    {
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

    pub fn run<Node, Cost, Direction, Position>(&self)
    where
        Maze: DirectionInstructor<SearchNode, Direction>
            + Graph<Node, Cost>
            + Graph<SearchNode, Cost>
            + GraphConverter<Node, SearchNode>,
        ISolver: Solver<Node, SearchNode, Cost, Maze>,
    {
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
