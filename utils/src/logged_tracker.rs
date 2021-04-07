extern crate std;

use components::robot::Tracker as ITracker;
use serde::Serialize;
use std::cell::RefCell;
use std::vec::Vec;

#[derive(Serialize)]
pub struct Log<State, Target> {
    state: State,
    target: Target,
}

pub struct JsonLoggedTracker<'a, State, Target, Tracker> {
    tracker: Tracker,
    logs: &'a RefCell<Vec<Log<State, Target>>>,
}

impl<'a, State, Target, Tracker> JsonLoggedTracker<'a, State, Target, Tracker> {
    pub fn new(tracker: Tracker, logs: &'a RefCell<Vec<Log<State, Target>>>) -> Self {
        Self { tracker, logs }
    }
}

impl<'a, Tracker, State, Target> ITracker<State, Target>
    for JsonLoggedTracker<'a, State, Target, Tracker>
where
    State: Clone,
    Target: Clone,
    Tracker: ITracker<State, Target>,
{
    type Error = Tracker::Error;

    fn track(&mut self, state: &State, target: &Target) -> Result<(), Self::Error> {
        self.logs.borrow_mut().push(Log {
            state: state.clone(),
            target: target.clone(),
        });
        self.tracker.track(state, target)
    }
}
