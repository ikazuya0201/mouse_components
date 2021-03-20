//! An implementation of [Robot](crate::agents::Robot) and its dependencies.

use core::marker::PhantomData;

use crate::agents::Robot as IRobot;

/// A trait that estimates the state of robot.
pub trait StateEstimator<Info> {
    type State;

    fn estimate(&mut self);
    fn state(&self) -> &Self::State;
    fn correct_state<Infos: IntoIterator<Item = Info>>(&mut self, infos: Infos);
}

/// A trait that tracks the given target with the given robot state.
pub trait Tracker<State, Target> {
    type Error;

    fn track(&mut self, state: &State, target: &Target) -> Result<(), Self::Error>;
}

/// A trait that detects obstacles, updates the states of wall and gives feedbacks for state
/// estimation.
pub trait WallDetector<State> {
    type Info;
    type Infos: IntoIterator<Item = Self::Info>;

    fn detect_and_update(&mut self, state: &State) -> Self::Infos;
}

/// An implementation of [Robot](crate::agents::Robot).
#[derive(Clone, PartialEq, Eq)]
pub struct Robot<Estimator, Tracker, Detector, State> {
    estimator: Estimator,
    tracker: Tracker,
    detector: Detector,
    _state: PhantomData<fn() -> State>,
}

impl<Estimator, Tracker, Detector, State> core::fmt::Debug
    for Robot<Estimator, Tracker, Detector, State>
where
    Estimator: core::fmt::Debug,
    Tracker: core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Robot")
            .field("estimator", &self.estimator)
            .field("tracker", &self.tracker)
            .finish()
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_track_and_update() {
        #[derive(PartialEq, Eq, Debug)]
        struct EstimatorType {
            state: usize,
            infos: Vec<usize>,
        }

        impl StateEstimator<usize> for EstimatorType {
            type State = usize;

            fn estimate(&mut self) {
                self.state += 1;
            }

            fn state(&self) -> &Self::State {
                &self.state
            }

            fn correct_state<Infos: IntoIterator<Item = usize>>(&mut self, infos: Infos) {
                self.infos = infos.into_iter().collect();
            }
        }

        #[derive(PartialEq, Eq, Debug)]
        struct DetectorType {
            state: Option<usize>,
            infos: Vec<usize>,
        }

        impl WallDetector<usize> for DetectorType {
            type Info = usize;
            type Infos = Vec<usize>;

            fn detect_and_update(&mut self, state: &usize) -> Self::Infos {
                self.state = Some(*state);
                self.infos = self
                    .infos
                    .clone()
                    .into_iter()
                    .map(|info| info + state)
                    .collect();
                self.infos.clone()
            }
        }

        #[derive(PartialEq, Eq, Debug)]
        struct TrackerType {
            state: Option<usize>,
            target: Option<usize>,
        }

        impl Tracker<usize, usize> for TrackerType {
            type Error = core::convert::Infallible;

            fn track(&mut self, state: &usize, target: &usize) -> Result<(), Self::Error> {
                self.state = Some(*state);
                self.target = Some(*target);
                Ok(())
            }
        }

        let mut robot = Robot::new(
            EstimatorType {
                state: 0,
                infos: Vec::new(),
            },
            TrackerType {
                state: None,
                target: None,
            },
            DetectorType {
                state: None,
                infos: vec![0, 1, 2],
            },
        );

        robot.track_and_update(&3).expect("This is a bug of test");
        let Robot {
            estimator,
            tracker,
            detector,
            ..
        } = robot;

        assert_eq!(
            estimator,
            EstimatorType {
                state: 1,
                infos: vec![1, 2, 3]
            }
        );
        assert_eq!(
            tracker,
            TrackerType {
                state: Some(1),
                target: Some(3)
            }
        );
        assert_eq!(
            detector,
            DetectorType {
                state: Some(1),
                infos: vec![1, 2, 3]
            }
        );
    }
}
