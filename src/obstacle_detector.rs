pub mod distance_sensor;

use generic_array::{ArrayLength, GenericArray};
use heapless::Vec;
use quantities::Distance;

use crate::agent::{self, Pose};
use crate::tracker::State;
use crate::utils::sample::Sample;
use distance_sensor::DistanceSensor;

#[derive(Clone, Debug, PartialEq)]
pub struct Obstacle {
    pub source: Pose,
    pub distance: Sample<Distance>,
}

pub struct ObstacleDetector<D, N>
where
    D: DistanceSensor,
    N: ArrayLength<D>,
{
    distance_sensors: GenericArray<D, N>,
}

impl<D, N> ObstacleDetector<D, N>
where
    D: DistanceSensor,
    N: ArrayLength<D>,
{
    pub fn new(distance_sensors: GenericArray<D, N>) -> Self {
        Self { distance_sensors }
    }
}

impl<D, N> agent::ObstacleDetector<Obstacle, State> for ObstacleDetector<D, N>
where
    D: DistanceSensor,
    N: ArrayLength<D> + ArrayLength<Obstacle>,
{
    type Obstacles = Vec<Obstacle, N>;

    fn detect(&mut self, state: State) -> Self::Obstacles {
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
    use quantities::Angle;

    use super::*;
    use crate::agent::Pose;

    struct IDistanceSensor {
        pose: Pose,
        distance: Option<Sample<Distance>>,
    }

    impl IDistanceSensor {
        fn new(pose: Pose, distance: Option<Sample<Distance>>) -> Self {
            Self { pose, distance }
        }
    }

    impl DistanceSensor for IDistanceSensor {
        type Error = ();

        fn pose(&self) -> Pose {
            self.pose
        }

        fn get_distance(&mut self) -> nb::Result<Sample<Distance>, Self::Error> {
            self.distance.clone().ok_or(nb::Error::Other(()))
        }
    }

    #[test]
    fn test_detect() {
        use generic_array::arr;

        let standard_deviation = Distance::from_meters(0.001);
        let sensors = arr![
            IDistanceSensor;
            IDistanceSensor::new(
                Pose {
                    x: Distance::from_meters(0.015),
                    y: Distance::from_meters(0.015),
                    theta: Angle::from_degree(0.0),
                },
                Some( Sample{
                    mean: Distance::from_meters(0.03),
                    standard_deviation,
                }),
            ),
            IDistanceSensor::new(
                Pose {
                    x: Distance::from_meters(-0.015),
                    y: Distance::from_meters(0.015),
                    theta: Angle::from_degree(180.0),
                },
                None,
            ),
            IDistanceSensor::new(
                Pose {
                    x: Distance::from_meters(0.0),
                    y: Distance::from_meters(0.025),
                    theta: Angle::from_degree(90.0),
                },
                Some(Sample{
                    mean:Distance::from_meters(0.02),
                    standard_deviation,
                }
                    ),
            ),
        ];
        let expected = vec![
            Pose {
                x: Distance::from_meters(0.015),
                y: Distance::from_meters(0.015),
                theta: Angle::from_degree(0.0),
            },
            Pose {
                x: Distance::from_meters(0.0),
                y: Distance::from_meters(0.025),
                theta: Angle::from_degree(90.0),
            },
        ];

        let mut detector = ObstacleDetector::new(sensors);
        let obstacles = agent::ObstacleDetector::detect(&mut detector, Default::default());
        for (obstacle, expected) in obstacles.into_iter().zip(expected.into_iter()) {
            assert_relative_eq!(obstacle.source.x.as_meters(), expected.x.as_meters());
            assert_relative_eq!(obstacle.source.y.as_meters(), expected.y.as_meters());
            assert_relative_eq!(
                obstacle.source.theta.as_radian(),
                expected.theta.as_radian()
            );
        }
    }
}
