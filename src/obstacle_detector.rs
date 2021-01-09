use heapless::Vec;
use uom::si::f32::Length;

use crate::agent;
use crate::data_types::Pose;
use crate::tracker::State;
use crate::utils::sample::Sample;

pub trait DistanceSensor {
    type Error;

    fn pose(&self) -> Pose;
    fn get_distance(&mut self) -> nb::Result<Sample<Length>, Self::Error>;
}

#[derive(Clone, Debug, PartialEq)]
pub struct Obstacle {
    pub source: Pose,
    pub distance: Sample<Length>,
}

type SensorSizeUpperBound = typenum::consts::U6;

//NOTE: the number of sensors is upper-bounded.
pub struct ObstacleDetector<D> {
    distance_sensors: Vec<D, SensorSizeUpperBound>,
}

impl<D> ObstacleDetector<D> {
    pub fn new<I: IntoIterator<Item = D>>(distance_sensors: I) -> Self {
        let distance_sensors = distance_sensors.into_iter().collect();
        Self { distance_sensors }
    }
}

impl<D> agent::ObstacleDetector<State> for ObstacleDetector<D>
where
    D: DistanceSensor,
{
    type Obstacle = Obstacle;
    type Obstacles = Vec<Obstacle, SensorSizeUpperBound>;

    fn detect(&mut self, state: &State) -> Self::Obstacles {
        let mut obstacles = Vec::new();
        for distance_sensor in &mut self.distance_sensors {
            let pose = distance_sensor.pose();
            if let Ok(distance) = distance_sensor.get_distance() {
                let x = state.x.x + pose.x;
                let y = state.y.x + pose.y;
                let theta = state.theta.x + pose.theta;
                let obstacle = Obstacle {
                    source: Pose { x, y, theta },
                    distance,
                };
                obstacles.push(obstacle).unwrap();
            }
        }
        obstacles
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;
    use crate::data_types::Pose;
    use uom::si::{angle::degree, f32::Angle, length::meter};

    struct IDistanceSensor {
        pose: Pose,
        distance: Option<Sample<Length>>,
    }

    impl IDistanceSensor {
        fn new(pose: Pose, distance: Option<Sample<Length>>) -> Self {
            Self { pose, distance }
        }
    }

    impl DistanceSensor for IDistanceSensor {
        type Error = ();

        fn pose(&self) -> Pose {
            self.pose
        }

        fn get_distance(&mut self) -> nb::Result<Sample<Length>, Self::Error> {
            self.distance.clone().ok_or(nb::Error::Other(()))
        }
    }

    #[test]
    fn test_detect() {
        let standard_deviation = Length::new::<meter>(0.001);
        let sensors = vec![
            IDistanceSensor::new(
                Pose {
                    x: Length::new::<meter>(0.015),
                    y: Length::new::<meter>(0.015),
                    theta: Angle::new::<degree>(0.0),
                },
                Some(Sample {
                    mean: Length::new::<meter>(0.03),
                    standard_deviation,
                }),
            ),
            IDistanceSensor::new(
                Pose {
                    x: Length::new::<meter>(-0.015),
                    y: Length::new::<meter>(0.015),
                    theta: Angle::new::<degree>(180.0),
                },
                None,
            ),
            IDistanceSensor::new(
                Pose {
                    x: Length::new::<meter>(0.0),
                    y: Length::new::<meter>(0.025),
                    theta: Angle::new::<degree>(90.0),
                },
                Some(Sample {
                    mean: Length::new::<meter>(0.02),
                    standard_deviation,
                }),
            ),
        ];
        let expected = vec![
            Pose {
                x: Length::new::<meter>(0.015),
                y: Length::new::<meter>(0.015),
                theta: Angle::new::<degree>(0.0),
            },
            Pose {
                x: Length::new::<meter>(0.0),
                y: Length::new::<meter>(0.025),
                theta: Angle::new::<degree>(90.0),
            },
        ];

        use crate::prelude::*;

        let mut detector = ObstacleDetector::new(sensors);
        let obstacles = detector.detect(&Default::default());
        for (obstacle, expected) in obstacles.into_iter().zip(expected.into_iter()) {
            assert_relative_eq!(obstacle.source.x.get::<meter>(), expected.x.get::<meter>());
            assert_relative_eq!(obstacle.source.y.get::<meter>(), expected.y.get::<meter>());
            assert_relative_eq!(
                obstacle.source.theta.get::<degree>(),
                expected.theta.get::<degree>()
            );
        }
    }
}
