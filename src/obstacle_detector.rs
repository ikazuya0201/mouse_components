//! An implementation of [ObstacleDetector](crate::wall_detector::ObstacleDetector).

use heapless::Vec;
#[allow(unused_imports)]
use micromath::F32Ext;
use serde::{Deserialize, Serialize};
use uom::si::f32::Length;

use crate::tracker::RobotState;
use crate::types::data::Pose;
use crate::utils::sample::Sample;
use crate::wall_detector::ObstacleDetector as IObstacleDetector;
use crate::{Construct, Deconstruct};

pub trait DistanceSensor {
    type Error;

    fn pose(&self) -> &Pose;
    fn get_distance(&mut self) -> nb::Result<Sample<Length>, Self::Error>;
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Obstacle {
    pub source: Pose,
    pub distance: Sample<Length>,
}

pub(crate) const SENSOR_SIZE_UPPER_BOUND: usize = 6;

const POSE_HISTORY_LIMIT: usize = 15;

//NOTE: the number of sensors is upper-bounded.
pub struct ObstacleDetector<D> {
    distance_sensors: Vec<D, SENSOR_SIZE_UPPER_BOUND>,
    pose_histories: Vec<Vec<Pose, POSE_HISTORY_LIMIT>, SENSOR_SIZE_UPPER_BOUND>,
}

impl<D> ObstacleDetector<D> {
    pub fn new<I: IntoIterator<Item = D>>(distance_sensors: I) -> Self {
        let distance_sensors: Vec<D, SENSOR_SIZE_UPPER_BOUND> =
            distance_sensors.into_iter().collect();
        let pose_histories = core::iter::repeat(Vec::new())
            .take(distance_sensors.len())
            .collect();
        Self {
            distance_sensors,
            pose_histories,
        }
    }

    pub fn release(self) -> Vec<D, SENSOR_SIZE_UPPER_BOUND> {
        let Self {
            distance_sensors, ..
        } = self;
        distance_sensors
    }
}

/// Resource for [ObstacleDetector].
#[derive(PartialEq, Eq, Debug)]
pub struct ObstacleDetectorResource<DistanceSensor> {
    pub distance_sensors: Vec<DistanceSensor, SENSOR_SIZE_UPPER_BOUND>,
}

impl<DistanceSensor, Config, State, Resource> Construct<Config, State, Resource>
    for ObstacleDetector<DistanceSensor>
where
    Resource: AsMut<Option<ObstacleDetectorResource<DistanceSensor>>>,
{
    fn construct<'a>(_config: &'a Config, _state: &'a State, resource: &'a mut Resource) -> Self {
        let ObstacleDetectorResource { distance_sensors } =
            resource.as_mut().take().unwrap_or_else(|| unimplemented!());
        Self::new(distance_sensors)
    }
}

impl<DistanceSensor, State, Resource> Deconstruct<State, Resource>
    for ObstacleDetector<DistanceSensor>
where
    State: Default,
    Resource: From<ObstacleDetectorResource<DistanceSensor>>,
{
    fn deconstruct(self) -> (State, Resource) {
        let distance_sensors = self.release();
        (
            Default::default(),
            ObstacleDetectorResource { distance_sensors }.into(),
        )
    }
}

impl<D> IObstacleDetector<RobotState> for ObstacleDetector<D>
where
    D: DistanceSensor,
{
    type Obstacle = Obstacle;
    type Obstacles = Vec<Obstacle, SENSOR_SIZE_UPPER_BOUND>;

    fn detect(&mut self, state: &RobotState) -> Self::Obstacles {
        let mut obstacles = Vec::new();
        let current_pose = Pose {
            x: state.x.x,
            y: state.y.x,
            theta: state.theta.x,
        };
        for (i, distance_sensor) in self.distance_sensors.iter_mut().enumerate() {
            if let Ok(distance) = distance_sensor.get_distance() {
                let sensor_pose = distance_sensor.pose();
                // TODO: use configurable value as index (e.g. `self.pose_histories[i].len() / val`)
                let machine_pose = self.pose_histories[i]
                    .get(0)
                    .cloned()
                    .unwrap_or(current_pose);
                let sin_th = machine_pose.theta.value.sin();
                let cos_th = machine_pose.theta.value.cos();
                let x = machine_pose.x + sensor_pose.x * cos_th - sensor_pose.y * sin_th;
                let y = machine_pose.y + sensor_pose.x * sin_th + sensor_pose.y * cos_th;
                let theta = machine_pose.theta + sensor_pose.theta;
                let obstacle = Obstacle {
                    source: Pose { x, y, theta },
                    distance,
                };
                obstacles.push(obstacle).unwrap();
                self.pose_histories[i].clear();
            } else {
                let _ = self.pose_histories[i].push(current_pose);
            }
        }
        obstacles
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use uom::si::{angle::degree, f32::Angle, length::meter};

    use super::*;
    use crate::types::data::Pose;

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

        fn pose(&self) -> &Pose {
            &self.pose
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
                    y: Length::new::<meter>(-0.015),
                    theta: Angle::new::<degree>(-90.0),
                },
                Some(Sample {
                    mean: Length::new::<meter>(0.03),
                    standard_deviation,
                }),
            ),
            IDistanceSensor::new(
                Pose {
                    x: Length::new::<meter>(0.015),
                    y: Length::new::<meter>(0.015),
                    theta: Angle::new::<degree>(90.0),
                },
                None,
            ),
            IDistanceSensor::new(
                Pose {
                    x: Length::new::<meter>(0.025),
                    y: Length::new::<meter>(0.0),
                    theta: Angle::new::<degree>(0.0),
                },
                Some(Sample {
                    mean: Length::new::<meter>(0.02),
                    standard_deviation,
                }),
            ),
        ];
        let expected = vec![
            Pose {
                x: Length::new::<meter>(0.06),
                y: Length::new::<meter>(0.06),
                theta: Angle::new::<degree>(0.0),
            },
            Pose {
                x: Length::new::<meter>(0.045),
                y: Length::new::<meter>(0.07),
                theta: Angle::new::<degree>(90.0),
            },
        ];

        use crate::prelude::*;
        use crate::types::data::{AngleState, LengthState};

        let mut detector = ObstacleDetector::new(sensors);
        let obstacles = detector.detect(&RobotState {
            x: LengthState {
                x: Length::new::<meter>(0.045),
                ..Default::default()
            },
            y: LengthState {
                x: Length::new::<meter>(0.045),
                ..Default::default()
            },
            theta: AngleState {
                x: Angle::new::<degree>(90.0),
                ..Default::default()
            },
        });
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
