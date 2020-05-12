mod agent;
mod counter;
mod maze;
mod mode;
mod operator;
mod solver;
mod switch;

use core::sync::atomic::Ordering;

pub use agent::Agent;
pub use counter::Counter;
pub use maze::{DirectionInstructor, Graph, GraphConverter, ObstacleInterpreter};
use mode::{AtomicMode, Mode};
pub use operator::Operator;
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
    SearchOperator: Operator,
{
    pub fn new(switch: ISwitch, counter: ICounter, search_operator: SearchOperator) -> Self {
        Self {
            mode: AtomicMode::new(Mode::Idle),
            switch,
            counter,
            search_operator,
        }
    }

    //called by periodic interrupt
    pub fn tick(&self) {
        use Mode::*;
        use Ordering::Relaxed;
        match self.mode.load(Relaxed) {
            Idle => (),
            Search => self.search_operator.tick(),
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
            if let Ok(mode) = match self.mode.load(Ordering::Relaxed) {
                Idle => Err(NotFinishError),
                Search => self.search_operator.run(),
                FastRun => self.fast_run(),
                Select => self.mode_select(),
            } {
                self.mode.store(mode, Ordering::Relaxed);
            }
        }
    }

    fn fast_run(&self) -> Result<Mode, NotFinishError> {
        Ok(Mode::Idle)
    }

    fn mode_select(&self) -> Result<Mode, NotFinishError> {
        self.counter.reset();
        let mut mode = Mode::Idle;
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
