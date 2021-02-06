use core::marker::PhantomData;

use crate::agents::Robot as IRobot;

/// A trait that estimates the state of robot.
pub trait StateEstimator<Info> {
    type State;

    fn init(&mut self);
    fn estimate(&mut self);
    fn state(&self) -> &Self::State;
    fn correct_state<Infos: IntoIterator<Item = Info>>(&mut self, infos: Infos);
}

/// A trait that tracks the given target with the given robot state.
pub trait Tracker<State, Target> {
    type Error;

    fn init(&mut self);
    fn track(&mut self, state: &State, target: &Target) -> Result<(), Self::Error>;
    fn stop(&mut self);
}

/// A trait that detects obstacles, updates the states of wall and gives feedbacks for state
/// estimation.
pub trait WallDetector<State> {
    type Info;
    type Infos: IntoIterator<Item = Self::Info>;

    fn detect_and_update(&mut self, state: &State) -> Self::Infos;
}

/// An implementation of [Robot](crate::agents::Robot).
pub struct Robot<Estimator, Tracker, Detector, State> {
    estimator: Estimator,
    tracker: Tracker,
    detector: Detector,
    _state: PhantomData<fn() -> State>,
}

impl<Estimator, Tracker, Detector, State> Robot<Estimator, Tracker, Detector, State> {
    /// Makes a new Robot with given args.
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
