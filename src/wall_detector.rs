//! An implementation of [WallDetector](crate::robot::WallDetector).

use alloc::rc::Rc;
use core::marker::PhantomData;

use heapless::Vec;
#[allow(unused_imports)]
use micromath::F32Ext;
use serde::{Deserialize, Serialize};
use uom::si::f32::Length;
use uom::si::{angle::revolution, length::meter};

use crate::impl_setter;
use crate::robot::WallDetector as IWallDetector;
use crate::types::data::{Pose, RobotState};
use crate::utils::{
    builder::{ok_or, BuilderResult},
    probability::Probability,
    random::Random,
    total::Total,
};
use crate::wall_manager::Wall;
use crate::{Construct, Deconstruct, Merge};

/// An info for wall state.
pub struct WallInfo<Wall> {
    pub wall: Wall,
    pub existing_distance: Length,
    pub not_existing_distance: Length,
}

/// A trait that manages existence probabilities of walls.
pub trait WallProbabilityManager<Wall>: Send + Sync {
    type Error;

    fn try_existence_probability(&self, wall: &Wall) -> Result<Probability, Self::Error>;
    fn try_update(&self, wall: &Wall, probablity: &Probability) -> Result<(), Self::Error>;
}

/// A trait that measures the distance from this sensor to an obstacle in front of the sensor.
pub trait DistanceSensor {
    type Error;

    /// Return the relative pose of the sensor from the center of the machine.
    /// The x axis is directed from the center of the machine to the front of the machine.
    fn pose(&self) -> &Pose;
    fn get_distance(&mut self) -> nb::Result<Random<Length>, Self::Error>;
}

pub(crate) const SENSOR_SIZE_UPPER_BOUND: usize = 6;

/// An implementation of [WallDetector](crate::robot::WallDetector).
pub struct WallDetector<Manager, DistanceSensor, const N: usize> {
    manager: Rc<Manager>,
    distance_sensors: Vec<DistanceSensor, SENSOR_SIZE_UPPER_BOUND>,
    past_poses: [Option<Pose>; SENSOR_SIZE_UPPER_BOUND],
    converter: PoseConverter<N>,
}

impl<Manager, DistanceSensor, const N: usize> WallDetector<Manager, DistanceSensor, N> {
    pub fn new(
        manager: Rc<Manager>,
        distance_sensors: Vec<DistanceSensor, SENSOR_SIZE_UPPER_BOUND>,
        converter: PoseConverter<N>,
    ) -> Self {
        Self {
            manager,
            distance_sensors,
            past_poses: [None; SENSOR_SIZE_UPPER_BOUND],
            converter,
        }
    }

    pub fn release(self) -> (Rc<Manager>, Vec<DistanceSensor, SENSOR_SIZE_UPPER_BOUND>) {
        let Self {
            distance_sensors,
            manager,
            ..
        } = self;
        (manager, distance_sensors)
    }
}

/// Config for [WallDetector].
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct WallDetectorConfig {
    pub square_width: Length,
    pub wall_width: Length,
    pub ignore_radius_from_pillar: Length,
    pub ignore_length_from_wall: Length,
}

pub struct WallDetectorResource<Manager, DistanceSensor> {
    pub wall_manager: Rc<Manager>,
    pub distance_sensors: Vec<DistanceSensor, SENSOR_SIZE_UPPER_BOUND>,
}

impl<Manager, DistanceSensor, Config, State, Resource, const N: usize>
    Construct<Config, State, Resource> for WallDetector<Manager, DistanceSensor, N>
where
    Config: AsRef<WallDetectorConfig>,
    Resource: AsMut<Option<WallDetectorResource<Manager, DistanceSensor>>>,
{
    fn construct<'a>(config: &'a Config, _state: &'a State, resource: &'a mut Resource) -> Self {
        let WallDetectorResource {
            wall_manager,
            distance_sensors,
        } = resource
            .as_mut()
            .take()
            .unwrap_or_else(|| unreachable!("This is a bug"));
        let config = config.as_ref();
        let converter = PoseConverterBuilder::new()
            .square_width(config.square_width)
            .wall_width(config.wall_width)
            .ignore_radius_from_pillar(config.ignore_radius_from_pillar)
            .ignore_length_from_wall(config.ignore_length_from_wall)
            .build()
            .expect("Should never panic");
        Self::new(wall_manager, distance_sensors, converter)
    }
}

impl<Manager, DistanceSensor, State, Resource, const N: usize> Deconstruct<State, Resource>
    for WallDetector<Manager, DistanceSensor, N>
where
    State: Default,
    Resource: Merge + From<WallDetectorResource<Manager, DistanceSensor>>,
{
    fn deconstruct(self) -> (State, Resource) {
        let (wall_manager, distance_sensors) = self.release();
        (
            Default::default(),
            WallDetectorResource {
                wall_manager,
                distance_sensors,
            }
            .into(),
        )
    }
}

impl<Manager, DistanceSensorType, const N: usize> IWallDetector<RobotState>
    for WallDetector<Manager, DistanceSensorType, N>
where
    Manager: WallProbabilityManager<Wall<N>>,
    DistanceSensorType: DistanceSensor,
{
    fn detect_and_update(&mut self, state: &mut RobotState) {
        use uom::si::ratio::ratio;

        let current_pose = Pose {
            x: state.x.x.mean,
            y: state.y.x.mean,
            theta: state.theta.x,
        };

        let mut delta_x = Length::default();
        let mut delta_y = Length::default();
        let average_standard_deviation =
            (state.x.x.standard_deviation + state.y.x.standard_deviation) / 2.0;
        let mut sigma2 = average_standard_deviation * average_standard_deviation;

        for (i, distance_sensor) in self.distance_sensors.iter_mut().enumerate() {
            let distance = if let Ok(distance) = distance_sensor.get_distance() {
                distance
            } else {
                if self.past_poses[i].is_none() {
                    self.past_poses[i] = Some(current_pose);
                }
                continue;
            };

            let sensor_relative_pose = distance_sensor.pose();
            // TODO: use configurable value as index (e.g. `self.pose_histories[i].len() / val`)
            let machine_pose = self.past_poses[i].take().unwrap_or(current_pose);
            let sin_th = machine_pose.theta.value.sin();
            let cos_th = machine_pose.theta.value.cos();

            debug_assert!(!machine_pose.x.is_nan());
            debug_assert!(!machine_pose.y.is_nan());
            debug_assert!(!sensor_relative_pose.x.is_nan());
            debug_assert!(!sensor_relative_pose.y.is_nan());

            let sensor_pose = Pose {
                x: machine_pose.x + sensor_relative_pose.x * cos_th
                    - sensor_relative_pose.y * sin_th
                    + delta_x,
                y: machine_pose.y
                    + sensor_relative_pose.x * sin_th
                    + sensor_relative_pose.y * cos_th
                    + delta_y,
                theta: machine_pose.theta + sensor_relative_pose.theta,
            };

            let wall_info = if let Ok(wall_info) = self.converter.convert(&sensor_pose) {
                wall_info
            } else {
                continue;
            };

            let existence =
                if let Ok(existence) = self.manager.try_existence_probability(&wall_info.wall) {
                    existence
                } else {
                    continue;
                };

            if existence.is_zero() {
                continue;
            } else if existence.is_one() {
                let sigma_sens2 = distance.standard_deviation * distance.standard_deviation;
                let k = (sigma2 / (sigma2 + sigma_sens2)).get::<ratio>();

                let distance_diff = k * (wall_info.existing_distance - distance.mean);
                let cos_th = sensor_pose.theta.value.cos();
                let sin_th = sensor_pose.theta.value.sin();
                delta_x += distance_diff * cos_th;
                delta_y += distance_diff * sin_th;
                sigma2 *= 1.0 - k;
                continue;
            }

            let exist_val = {
                let tmp = ((wall_info.existing_distance - distance.mean)
                    / distance.standard_deviation)
                    .get::<ratio>();
                -tmp * tmp / 2.0
            };
            let not_exist_val = {
                let tmp = ((wall_info.not_existing_distance - distance.mean)
                    / distance.standard_deviation)
                    .get::<ratio>();
                -tmp * tmp / 2.0
            };

            debug_assert!(!exist_val.is_nan());
            debug_assert!(!not_exist_val.is_nan());

            let min = if exist_val < not_exist_val {
                exist_val
            } else {
                not_exist_val
            };
            let exist_val = (exist_val - min).exp() * existence;
            let not_exist_val = (not_exist_val - min).exp() * existence.reverse();

            let existence = if exist_val.is_infinite() {
                Probability::one()
            } else if not_exist_val.is_infinite() {
                Probability::zero()
            } else {
                Probability::new(exist_val / (exist_val + not_exist_val))
                    .unwrap_or_else(|err| unreachable!("Should never be out of bound: {:?}", err))
            };

            // Ignore the result.
            let _ = self.manager.try_update(&wall_info.wall, &existence);
        }
        // Update state.
        state.x.x.mean += delta_x;
        state.y.x.mean += delta_y;
        let standard_deviation = crate::utils::math::sqrt(sigma2);
        state.x.x.standard_deviation = standard_deviation;
        state.y.x.standard_deviation = standard_deviation;
    }
}

/// An implementation of [PoseConverter](crate::wall_detector::PoseConverter).
#[derive(Clone, PartialEq, Debug)]
pub struct PoseConverter<const N: usize> {
    i_square_width: i32, //[mm]
    square_width_half: Length,
    square_width: Length,
    ignore_radius_from_pillar: Length,
    ignore_length_from_wall: Length,
    p1: Length,
    p2: Length,
    n1: Length,
    n2: Length,
}

pub(crate) const DEFAULT_SQUARE_WIDTH: Length = Length {
    dimension: PhantomData,
    units: PhantomData,
    value: 0.09,
};
pub(crate) const DEFAULT_WALL_WIDTH: Length = Length {
    dimension: PhantomData,
    units: PhantomData,
    value: 0.006,
};
pub(crate) const DEFAULT_IGNORE_RADIUS: Length = Length {
    dimension: PhantomData,
    units: PhantomData,
    value: 0.01,
};
pub(crate) const DEFAULT_IGNORE_LENGTH: Length = Length {
    value: 0.008,
    dimension: PhantomData,
    units: PhantomData,
};

impl<const N: usize> Default for PoseConverter<N> {
    fn default() -> Self {
        Self::new(
            DEFAULT_SQUARE_WIDTH,
            DEFAULT_WALL_WIDTH,
            DEFAULT_IGNORE_RADIUS,
            DEFAULT_IGNORE_LENGTH,
        )
    }
}

impl<const N: usize> PoseConverter<N> {
    fn new(
        square_width: Length,
        wall_width: Length,
        ignore_radius_from_pillar: Length,
        ignore_length_from_wall: Length,
    ) -> Self {
        let p1 = square_width - wall_width / 2.0;
        let p2 = p1 + square_width;
        let n1 = wall_width / 2.0;
        let n2 = n1 - square_width;
        Self {
            i_square_width: (square_width.get::<meter>() * 1000.0) as i32,
            square_width_half: square_width / 2.0,
            square_width,
            ignore_radius_from_pillar,
            ignore_length_from_wall,
            p1,
            p2,
            n1,
            n2,
        }
    }

    //(remainder, quotient)
    fn remquof_with_width(&self, val: Length) -> (Length, i32) {
        let quo = (val.get::<meter>() * 1000.0) as i32 / self.i_square_width;
        let rem = val - Length::new::<meter>((quo * self.i_square_width) as f32 * 0.001);
        (rem, quo)
    }
}

impl<const N: usize> PoseConverter<N> {
    fn is_near_pillar(&self, pose: &Pose) -> bool {
        let rot = pose.theta.get::<revolution>().rem_euclid(1.0);
        let pillars = if !(0.125..=0.875).contains(&rot) {
            [
                (self.square_width, Length::default()),
                (self.square_width, self.square_width),
            ]
        } else if rot < 0.375 {
            [
                (self.square_width, self.square_width),
                (Length::default(), self.square_width),
            ]
        } else if rot < 0.625 {
            [
                (Length::default(), self.square_width),
                (Length::default(), Length::default()),
            ]
        } else {
            [
                (Length::default(), Length::default()),
                (self.square_width, Length::default()),
            ]
        };

        let sin_th = pose.theta.value.sin();
        let cos_th = pose.theta.value.cos();
        let mind = pillars
            .iter()
            .map(|pillar| {
                Total((sin_th * (pose.x - pillar.0) - cos_th * (pose.y - pillar.1)).abs())
            })
            .min()
            .expect("Should never panic")
            .0;

        mind < self.ignore_radius_from_pillar
    }
}

/// Error on [PoseConverter](PoseConverter).
#[derive(Clone, PartialEq, Debug)]
pub struct OutOfBoundError {
    pose: Pose,
}

#[derive(Clone, PartialEq, Debug)]
pub enum ConversionError {
    WallCreation(crate::wall_manager::OutOfBoundError),
    OutOfBound(OutOfBoundError),
}

impl From<crate::wall_manager::OutOfBoundError> for ConversionError {
    fn from(value: crate::wall_manager::OutOfBoundError) -> Self {
        ConversionError::WallCreation(value)
    }
}

impl From<OutOfBoundError> for ConversionError {
    fn from(value: OutOfBoundError) -> Self {
        ConversionError::OutOfBound(value)
    }
}

impl<const N: usize> PoseConverter<N> {
    pub fn convert(&self, pose: &Pose) -> Result<WallInfo<Wall<N>>, ConversionError> {
        #[derive(Clone, Copy, Debug)]
        enum Axis {
            X(Length),
            Y(Length),
        }

        #[derive(Clone, Copy, Debug)]
        enum Direction {
            Right,
            Left,
            Top,
            Bottom,
        }

        use Direction::*;

        let create_error = || Err(ConversionError::OutOfBound(OutOfBoundError { pose: *pose }));

        debug_assert!(!pose.x.value.is_nan());
        debug_assert!(!pose.y.value.is_nan());

        let (x_rem, x_quo) = self.remquof_with_width(pose.x);
        let (y_rem, y_quo) = self.remquof_with_width(pose.y);

        let pose = Pose {
            x: x_rem,
            y: y_rem,
            theta: pose.theta,
        };

        if self.is_near_pillar(&pose) {
            return create_error();
        }

        if x_rem <= self.n1 || x_rem >= self.p1 || y_rem <= self.n1 || y_rem >= self.p1 {
            return create_error();
        }

        let rot = pose.theta.get::<revolution>().rem_euclid(1.0);

        let axes = if rot < 0.25 {
            [
                (Axis::X(self.p1), Right),
                (Axis::X(self.p2), Right),
                (Axis::Y(self.p1), Top),
                (Axis::Y(self.p2), Top),
            ]
        } else if rot < 0.5 {
            [
                (Axis::X(self.n1), Left),
                (Axis::X(self.n2), Left),
                (Axis::Y(self.p1), Top),
                (Axis::Y(self.p2), Top),
            ]
        } else if rot < 0.75 {
            [
                (Axis::X(self.n1), Left),
                (Axis::X(self.n2), Left),
                (Axis::Y(self.n1), Bottom),
                (Axis::Y(self.n2), Bottom),
            ]
        } else {
            [
                (Axis::X(self.p1), Right),
                (Axis::X(self.p2), Right),
                (Axis::Y(self.n1), Bottom),
                (Axis::Y(self.n2), Bottom),
            ]
        };

        let sin_th = pose.theta.value.sin();
        let cos_th = pose.theta.value.cos();

        let mut axes_distance = axes
            .iter()
            .map(|&(axis, direction)| {
                let dist = match axis {
                    Axis::X(x) => (x - x_rem) / cos_th,
                    Axis::Y(y) => (y - y_rem) / sin_th,
                };
                //assign infinity to invalid values
                let dist = if dist.get::<meter>() < 0.0 || dist.get::<meter>().is_nan() {
                    Length::new::<meter>(core::f32::INFINITY)
                } else {
                    dist
                };
                (direction, dist)
            })
            .collect::<Vec<_, 4>>();

        axes_distance.sort_unstable_by_key(|e| Total(e.1));

        let wall = match axes_distance[0].0 {
            Right => {
                if x_quo < 0 || y_quo < 0 {
                    return create_error();
                }
                Wall::new(x_quo as u8, y_quo as u8, false)
            }
            Left => {
                if x_quo < 1 || y_quo < 0 {
                    return create_error();
                }
                Wall::new((x_quo - 1) as u8, y_quo as u8, false)
            }
            Top => {
                if x_quo < 0 || y_quo < 0 {
                    return create_error();
                }
                Wall::new(x_quo as u8, y_quo as u8, true)
            }
            Bottom => {
                if x_quo < 0 || y_quo < 1 {
                    return create_error();
                }
                Wall::new(x_quo as u8, (y_quo - 1) as u8, true)
            }
        }?;

        let existing_distance = axes_distance[0].1; //the nearest one
        let not_existing_distance = axes_distance[1].1; //the 2nd nearest one

        if existing_distance <= self.ignore_length_from_wall {
            return create_error();
        }

        Ok(WallInfo {
            wall,
            existing_distance,
            not_existing_distance,
        })
    }
}

pub struct PoseConverterBuilder {
    square_width: Option<Length>,
    wall_width: Option<Length>,
    ignore_radius_from_pillar: Option<Length>,
    ignore_length_from_wall: Option<Length>,
}

impl Default for PoseConverterBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl PoseConverterBuilder {
    pub fn new() -> Self {
        Self {
            square_width: None,
            wall_width: None,
            ignore_radius_from_pillar: None,
            ignore_length_from_wall: None,
        }
    }

    impl_setter!(square_width: Length);
    impl_setter!(wall_width: Length);
    impl_setter!(ignore_length_from_wall: Length);
    impl_setter!(ignore_radius_from_pillar: Length);

    pub fn build<const N: usize>(&mut self) -> BuilderResult<PoseConverter<N>> {
        Ok(PoseConverter::new(
            self.square_width.unwrap_or(DEFAULT_SQUARE_WIDTH),
            self.wall_width.unwrap_or(DEFAULT_WALL_WIDTH),
            self.ignore_radius_from_pillar
                .unwrap_or(DEFAULT_IGNORE_RADIUS),
            self.ignore_length_from_wall
                .unwrap_or(DEFAULT_IGNORE_LENGTH),
        ))
    }
}

pub struct WallDetectorBuilder<Manager, DistanceSensor> {
    wall_manager: Option<Rc<Manager>>,
    distance_sensors: Option<Vec<DistanceSensor, SENSOR_SIZE_UPPER_BOUND>>,
    square_width: Option<Length>,
    wall_width: Option<Length>,
    ignore_radius_from_pillar: Option<Length>,
    ignore_length_from_wall: Option<Length>,
}

impl<Manager, DistanceSensor> Default for WallDetectorBuilder<Manager, DistanceSensor> {
    fn default() -> Self {
        Self::new()
    }
}

impl<Manager, DistanceSensor> WallDetectorBuilder<Manager, DistanceSensor> {
    pub fn new() -> Self {
        Self {
            wall_manager: None,
            distance_sensors: None,
            square_width: None,
            wall_width: None,
            ignore_radius_from_pillar: None,
            ignore_length_from_wall: None,
        }
    }

    impl_setter! {
        /// **Required**,
        /// Sets a reference to an implementation of [WallProbabilityManager](WallProbabilityManager).
        wall_manager: Rc<Manager>
    }

    impl_setter! {
        /// **Optional**,
        /// Sets the width of a square of a maze.
        ///
        /// Default: 90 \[mm\]
        square_width: Length
    }

    impl_setter! {
        /// **Optional**,
        /// Sets the width of a wall of a maze.
        ///
        /// Default: 6 \[mm\]
        wall_width: Length
    }

    impl_setter! {
        /// **Optional**,
        /// Sets a radius. If the position of a distance sensor is near than this radius, the
        /// result of the sensor will be ignored.
        ///
        /// Default: 10 \[mm\]
        ignore_radius_from_pillar: Length
    }

    impl_setter! {
        /// **Optional**,
        /// Sets a length. If the position of a distance sensor is near than this length, the
        /// result of the sensor will be ignored.
        ///
        /// Default: 8 \[mm\]
        ignore_length_from_wall: Length
    }

    impl_setter! {
        distance_sensors: Vec<DistanceSensor, SENSOR_SIZE_UPPER_BOUND>
    }

    pub fn build<const N: usize>(
        &mut self,
    ) -> BuilderResult<WallDetector<Manager, DistanceSensor, N>> {
        let converter = PoseConverter::<N>::new(
            self.square_width.unwrap_or(DEFAULT_SQUARE_WIDTH),
            self.wall_width.unwrap_or(DEFAULT_WALL_WIDTH),
            self.ignore_radius_from_pillar
                .unwrap_or(DEFAULT_IGNORE_RADIUS),
            self.ignore_length_from_wall
                .unwrap_or(DEFAULT_IGNORE_LENGTH),
        );
        Ok(WallDetector::new(
            ok_or(self.wall_manager.take(), "wall_manager")?,
            ok_or(self.distance_sensors.take(), "distance_sensors")?,
            converter,
        ))
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use uom::si::angle::degree;
    use uom::si::f32::Angle;

    use super::*;
    use crate::types::data::Pose;

    macro_rules! define_convert_ok_test {
        ($name:ident: ($size:expr, $value: expr)) => {
            #[test]
            fn $name() {
                let (input, expected) = $value;
                let input = Pose {
                    x: Length::new::<meter>(input.0),
                    y: Length::new::<meter>(input.1),
                    theta: Angle::new::<degree>(input.2),
                };
                let expected = WallInfo {
                    wall: Wall::<$size>::new(expected.0, expected.1, expected.2).unwrap(),
                    existing_distance: Length::new::<meter>(expected.3),
                    not_existing_distance: Length::new::<meter>(expected.4),
                };

                let converter = PoseConverter::<$size>::new(
                    Length::new::<meter>(0.09),
                    Length::new::<meter>(0.006),
                    Length::new::<meter>(0.01),
                    Length::new::<meter>(0.008),
                );
                let info = converter.convert(&input).unwrap();
                assert_eq!(info.wall, expected.wall);
                assert_relative_eq!(
                    info.existing_distance.get::<meter>(),
                    expected.existing_distance.get::<meter>(),
                );
                assert_relative_eq!(
                    info.not_existing_distance.get::<meter>(),
                    expected.not_existing_distance.get::<meter>(),
                );
            }
        };
    }

    macro_rules! define_convert_err_test {
        ($name:ident: ($size:expr, $value: expr)) => {
            #[test]
            fn $name() {
                let input = $value;
                let input = Pose {
                    x: Length::new::<meter>(input.0),
                    y: Length::new::<meter>(input.1),
                    theta: Angle::new::<degree>(input.2),
                };

                let converter = PoseConverter::<$size>::new(
                    Length::new::<meter>(0.09),
                    Length::new::<meter>(0.006),
                    Length::new::<meter>(0.01),
                    Length::new::<meter>(0.008),
                );
                assert!(converter.convert(&input).is_err());
            }
        };
    }

    define_convert_ok_test! (
    convert_ok_test1: (4, (
        (0.045, 0.045, 0.0),
        (0, 0, false, 0.042, 0.132),
    )));
    define_convert_ok_test! (
    convert_ok_test2: (4, (
        (0.077, 0.045, 45.0),
        (0, 0, false, 2.0f32.sqrt() * 0.01, 2.0f32.sqrt() * 0.042),
    )));
    define_convert_ok_test! (
    convert_ok_test3: (4, (
        (0.135, 0.045, 180.0),
        (0, 0, false, 0.042, 0.132),
    )));
    define_convert_ok_test! (
    convert_ok_test4: (4, (
        (0.045, 0.135, 270.0),
        (0, 0, true, 0.042, 0.132),
    )));
    define_convert_ok_test! (
    convert_ok_test5: (4, (
        (0.135, 0.135, 90.0),
        (1, 1, true, 0.042, 0.132),
    )));
    define_convert_ok_test! (
    convert_ok_test6: (4, (
        (0.135, 0.135, 180.0),
        (0, 1, false, 0.042, 0.132),
    )));
    define_convert_ok_test! (
    convert_ok_test7: (4, (
        (0.045, 0.135, 0.0),
        (0, 1, false, 0.042, 0.132),
    )));

    define_convert_err_test!(convert_err_test1: (4, (0.0, 0.0, 0.0)));
    define_convert_err_test!(convert_err_test2: (4, (0.405, 0.045, 0.0)));
    define_convert_err_test!(convert_err_test3: (4, (-0.045, 0.045, 0.0)));
    define_convert_err_test!(convert_err_test4: (4, (0.045, 0.045, 45.0)));
    define_convert_err_test!(convert_err_test5: (4, (0.085, 0.045, 0.0)));
}
