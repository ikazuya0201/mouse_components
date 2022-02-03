//! An implementation of [Robot](crate::agents::Robot) and its dependencies.

use crate::agents::Robot as IRobot;
use crate::{Construct, Deconstruct, Merge};

/// A trait that estimates the state of robot.
pub trait Estimator {
    type State;

    fn estimate(&mut self);
    fn state(&self) -> &Self::State;

    /// Returns mutable reference of estimated state.
    fn state_mut(&mut self) -> &mut Self::State;
}

/// A trait that tracks the given target with the given robot state.
pub trait Tracker<State, Target> {
    type Error;

    fn track(&mut self, state: &State, target: &Target) -> Result<(), Self::Error>;
    /// Stops all actuators in tracker.
    fn stop(&mut self);
}

/// A trait that detects obstacles, updates the states of wall and gives feedbacks for state
/// estimation.
pub trait WallDetector<State> {
    /// Updates walls by a given state and corrects the given state.
    fn detect_and_update(&mut self, state: &mut State);
}

/// An implementation of [Robot](crate::agents::Robot).
#[derive(Clone, PartialEq, Eq)]
pub struct Robot<Estimator, Tracker, Detector> {
    estimator: Estimator,
    tracker: Tracker,
    detector: Detector,
}

impl<Estimator, Tracker, Detector> core::fmt::Debug for Robot<Estimator, Tracker, Detector>
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

impl<Estimator, Tracker, Detector> Robot<Estimator, Tracker, Detector> {
    /// Makes a new Robot with given args.
    pub fn new(estimator: Estimator, tracker: Tracker, detector: Detector) -> Self {
        Self {
            estimator,
            tracker,
            detector,
        }
    }

    pub fn release(self) -> (Estimator, Tracker, Detector) {
        let Self {
            estimator,
            tracker,
            detector,
            ..
        } = self;
        (estimator, tracker, detector)
    }
}

impl<Estimator, Tracker, Detector, Config, State, Resource> Construct<Config, State, Resource>
    for Robot<Estimator, Tracker, Detector>
where
    Estimator: Construct<Config, State, Resource>,
    Tracker: Construct<Config, State, Resource>,
    Detector: Construct<Config, State, Resource>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let estimator = Estimator::construct(config, state, resource);
        let tracker = Tracker::construct(config, state, resource);
        let detector = Detector::construct(config, state, resource);
        Self::new(estimator, tracker, detector)
    }
}

impl<Estimator, Tracker, Detector, State, Resource> Deconstruct<State, Resource>
    for Robot<Estimator, Tracker, Detector>
where
    Estimator: Deconstruct<State, Resource>,
    Tracker: Deconstruct<State, Resource>,
    Detector: Deconstruct<State, Resource>,
    State: Merge,
    Resource: Merge,
{
    fn deconstruct(self) -> (State, Resource) {
        let (estimator, tracker, detector) = self.release();
        let (estimator_state, estimator_resource) = estimator.deconstruct();
        let (tracker_state, tracker_resource) = tracker.deconstruct();
        let (detector_state, detector_resource) = detector.deconstruct();
        (
            estimator_state.merge(tracker_state).merge(detector_state),
            estimator_resource
                .merge(tracker_resource)
                .merge(detector_resource),
        )
    }
}

impl<EstimatorType, TrackerType, Detector, Target> IRobot<Target>
    for Robot<EstimatorType, TrackerType, Detector>
where
    EstimatorType: Estimator,
    TrackerType: Tracker<EstimatorType::State, Target>,
    Detector: WallDetector<EstimatorType::State>,
{
    type Error = TrackerType::Error;

    fn track_and_update(&mut self, target: &Target) -> Result<(), Self::Error> {
        self.estimator.estimate();
        self.tracker.track(self.estimator.state(), target)?;
        self.detector.detect_and_update(self.estimator.state_mut());
        Ok(())
    }

    fn stop(&mut self) {
        self.tracker.stop();
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
        }

        impl Estimator for EstimatorType {
            type State = usize;

            fn estimate(&mut self) {
                self.state += 1;
            }

            fn state(&self) -> &Self::State {
                &self.state
            }

            fn state_mut(&mut self) -> &mut Self::State {
                &mut self.state
            }
        }

        #[derive(PartialEq, Eq, Debug)]
        struct DetectorType {
            state: Option<usize>,
        }

        impl WallDetector<usize> for DetectorType {
            fn detect_and_update(&mut self, state: &mut usize) {
                self.state = Some(*state);
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

            fn stop(&mut self) {
                unimplemented!()
            }
        }

        let mut robot = Robot::new(
            EstimatorType { state: 0 },
            TrackerType {
                state: None,
                target: None,
            },
            DetectorType { state: None },
        );

        robot.track_and_update(&3).expect("This is a bug of test");
        let Robot {
            estimator,
            tracker,
            detector,
            ..
        } = robot;

        assert_eq!(estimator, EstimatorType { state: 1 });
        assert_eq!(
            tracker,
            TrackerType {
                state: Some(1),
                target: Some(3)
            }
        );
        assert_eq!(detector, DetectorType { state: Some(1) });
    }
}
