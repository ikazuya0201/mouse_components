extern crate std;

use components::robot::Tracker as ITracker;
use serde::Serialize;

pub struct JsonLoggedTracker<Logger, Tracker> {
    logger: Logger,
    tracker: Tracker,
}

impl<Logger, Tracker> JsonLoggedTracker<Logger, Tracker> {
    pub fn new(logger: Logger, tracker: Tracker) -> Self {
        Self { logger, tracker }
    }
}

impl<Logger, Tracker, State, Target> ITracker<State, Target> for JsonLoggedTracker<Logger, Tracker>
where
    Logger: std::io::Write,
    Tracker: ITracker<State, Target>,
    State: Serialize,
    Target: Serialize,
{
    type Error = Tracker::Error;

    fn track(&mut self, state: &State, target: &Target) -> Result<(), Self::Error> {
        #[derive(Serialize)]
        struct Log<'a, State, Target> {
            state: &'a State,
            target: &'a Target,
        }

        self.logger
            .write_all(
                std::format!(
                    "{},",
                    serde_json::to_string(&Log { state, target })
                        .unwrap()
                        .as_str()
                )
                .as_bytes(),
            )
            .unwrap();
        self.tracker.track(state, target)
    }
}
