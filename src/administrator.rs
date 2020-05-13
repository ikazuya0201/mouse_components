mod agent;
mod counter;
mod maze;
mod mode;
pub mod operator;
mod solver;
mod switch;

use core::sync::atomic::Ordering::Relaxed;

pub use agent::Agent;
pub use counter::Counter;
pub use maze::{DirectionInstructor, Graph, GraphConverter, ObstacleInterpreter};
use mode::{AtomicMode, FastRun, Idle, Mode, Search, Select};
use operator::Operator;
pub use solver::Solver;
pub use switch::Switch;

#[derive(Debug, Clone, Copy)]
pub struct NotFinishError;

pub struct Administrator<ISwitch, ICounter, SearchOperator> {
    mode: AtomicMode,
    switch: ISwitch,
    counter: ICounter,
    search_operator: SearchOperator,
}

impl<ISwitch, ICounter, SearchOperator> Administrator<ISwitch, ICounter, SearchOperator>
where
    ISwitch: Switch,
    ICounter: Counter,
    SearchOperator: Operator<Search>,
{
    pub fn new(switch: ISwitch, counter: ICounter, search_operator: SearchOperator) -> Self {
        Self {
            mode: AtomicMode::new(Mode::Idle(Idle)),
            switch,
            counter,
            search_operator,
        }
    }

    //called by periodic interrupt
    pub fn tick(&self) {
        match self.mode.load(Relaxed) {
            Mode::Idle(_) => (),
            Mode::Search(_) => self.search_operator.tick(),
            Mode::Select(_) => return,
            _ => (),
        }
        self.store_if_switch_enabled(Mode::Select(Select));
    }

    fn store_if_switch_enabled(&self, mode: Mode) {
        if self.switch.is_enabled() {
            self.mode.store(mode, Relaxed);
        }
    }

    pub fn run(&self) {
        loop {
            if let Ok(mode) = match self.mode.load(Relaxed) {
                Mode::Idle(_) => Err(NotFinishError),
                Mode::Search(_) => self.search_operator.run(),
                Mode::FastRun(_) => self.fast_run(),
                Mode::Select(_) => self.mode_select(),
            } {
                self.mode.store(mode, Relaxed);
            }
        }
    }

    fn fast_run(&self) -> Result<Mode, NotFinishError> {
        Ok(Mode::Idle(Idle))
    }

    fn mode_select(&self) -> Result<Mode, NotFinishError> {
        self.counter.reset();
        let mut mode = Mode::Idle(Idle);
        //waiting for switch off
        while self.switch.is_enabled() {}

        while !self.switch.is_enabled() {
            let count = self.counter.count();
            mode = Mode::from(count % Mode::size());
        }

        while self.switch.is_enabled() {}

        Ok(mode)
    }
}
