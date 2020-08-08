pub mod distance_sensor;

use generic_array::{ArrayLength, GenericArray};
use heapless::Vec;
use quantities::Distance;

use crate::agent;
use crate::tracker::State;
use distance_sensor::DistanceSensor;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Obstacle {
    pub x: Distance,
    pub y: Distance,
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
                    x: x + distance * theta.cos(),
                    y: y + distance * theta.sin(),
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
        distance: Option<Distance>,
    }

    impl IDistanceSensor {
        fn new(pose: Pose, distance: Option<Distance>) -> Self {
            Self { pose, distance }
        }
    }

    impl DistanceSensor for IDistanceSensor {
        type Error = ();

        fn pose(&self) -> Pose {
            self.pose
        }

        fn get_distance(&mut self) -> nb::Result<Distance, Self::Error> {
            self.distance.ok_or(nb::Error::Other(()))
        }
    }

    #[test]
    fn test_detect() {
        use generic_array::arr;

        let sensors = arr![
            IDistanceSensor;
            IDistanceSensor::new(
                Pose {
                    x: Distance::from_meters(0.015),
                    y: Distance::from_meters(0.015),
                    theta: Angle::from_degree(0.0),
                },
                Some(Distance::from_meters(0.03)),
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
                Some(Distance::from_meters(0.02)),
            ),
        ];
        let expected = vec![
            Obstacle {
                x: Distance::from_meters(0.045),
                y: Distance::from_meters(0.015),
            },
            Obstacle {
                x: Distance::from_meters(0.0),
                y: Distance::from_meters(0.045),
            },
        ];

        let mut detector = ObstacleDetector::new(sensors);
        let obstacles = agent::ObstacleDetector::detect(&mut detector, Default::default());
        for (obstacle, expected) in obstacles.into_iter().zip(expected.into_iter()) {
            assert_relative_eq!(obstacle.x.as_meters(), expected.x.as_meters());
            assert_relative_eq!(obstacle.y.as_meters(), expected.y.as_meters());
        }
    }
}
