use core::marker::PhantomData;

use crate::agents::Robot as IRobot;

pub trait StateEstimator<Diff> {
    type State;

    fn init(&mut self);
    fn estimate(&mut self);
    fn state(&self) -> &Self::State;
    fn correct_state<Diffs: IntoIterator<Item = Diff>>(&mut self, diffs: Diffs);
}

pub trait Tracker<State, Target> {
    type Error;

    fn init(&mut self);
    fn track(&mut self, state: &State, target: &Target) -> Result<(), Self::Error>;
    fn stop(&mut self);
}

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

impl<Estimator, TrackerType, Detector, Target, State> IRobot<Target>
    for Robot<Estimator, TrackerType, Detector, State>
where
    Estimator: StateEstimator<Detector::Info, State = State>,
    TrackerType: Tracker<State, Target>,
    Detector: WallDetector<State>,
{
    type Error = TrackerType::Error;

    fn track_and_update(&mut self, target: &Target) -> Result<(), Self::Error> {
        self.estimator.estimate();
        let infos = self.detector.detect_and_update(self.estimator.state());
        self.estimator.correct_state(infos);
        self.tracker.track(self.estimator.state(), target)
    }
}
