use core::marker::PhantomData;

use crate::agents::search_agent::Robot as IRobot;
use crate::agents::{StateEstimator as IEstimator, Tracker as ITracker};

pub trait WallDetector<State> {
    type Info;
    type Infos: IntoIterator<Item = Self::Info>;

    fn detect_and_update(&mut self, state: &State) -> Self::Infos;
}

pub struct Robot<Estimator, Tracker, Detector, State> {
    estimator: Estimator,
    tracker: Tracker,
    detector: Detector,
    _state: PhantomData<fn() -> State>,
}

impl<Estimator, Tracker, Detector, State> Robot<Estimator, Tracker, Detector, State> {
    pub fn new(estimator: Estimator, tracker: Tracker, detector: Detector) -> Self {
        Self {
            estimator,
            tracker,
            detector,
            _state: PhantomData,
        }
    }
}

impl<Estimator, Tracker, Detector, Target, State> IRobot<Target>
    for Robot<Estimator, Tracker, Detector, State>
where
    Estimator: IEstimator<Detector::Info, State = State>,
    Tracker: ITracker<State, Target>,
    Detector: WallDetector<State>,
{
    type Error = Tracker::Error;

    fn track_and_update(&mut self, target: &Target) -> Result<(), Self::Error> {
        self.estimator.estimate();
        let infos = self.detector.detect_and_update(self.estimator.state());
        self.estimator.correct_state(infos);
        self.tracker.track(self.estimator.state(), target)
    }
}
