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
use crate::types::data::{Obstacle, Pose};
use crate::utils::{
    builder::{ok_or, BuilderResult},
    forced_vec::ForcedVec,
    probability::Probability,
    total::Total,
};
use crate::wall_manager::Wall;
use crate::{Construct, Deconstruct, Merge};

/// An info for corrects the state of robot.
#[derive(Debug, Clone)]
pub struct CorrectInfo {
    pub obstacle: Obstacle,
    pub diff_from_expected: Length,
}

/// An info for wall state.
pub struct WallInfo<Wall> {
    pub wall: Wall,
    pub existing_distance: Length,
    pub not_existing_distance: Length,
}

/// A trait that detects obstacle.
pub trait ObstacleDetector<State> {
    type Obstacle;
    type Obstacles: IntoIterator<Item = Self::Obstacle>;

    fn detect(&mut self, state: &State) -> Self::Obstacles;
}

/// A trait that manages existence probabilities of walls.
pub trait WallProbabilityManager<Wall>: Send + Sync {
    type Error;

    fn try_existence_probability(&self, wall: &Wall) -> Result<Probability, Self::Error>;
    fn try_update(&self, wall: &Wall, probablity: &Probability) -> Result<(), Self::Error>;
}

/// An implementation of [WallDetector](crate::robot::WallDetector).
pub struct WallDetector<Manager, Detector, const N: usize> {
    manager: Rc<Manager>,
    detector: Detector,
    converter: PoseConverter<N>,
}

impl<Manager, Detector, const N: usize> WallDetector<Manager, Detector, N> {
    pub fn new(manager: Rc<Manager>, detector: Detector, converter: PoseConverter<N>) -> Self {
        Self {
            manager,
            detector,
            converter,
        }
    }

    pub fn release(self) -> (Rc<Manager>, Detector) {
        let Self {
            detector, manager, ..
        } = self;
        (manager, detector)
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

impl<Manager, Detector, Config, State, Resource, const N: usize> Construct<Config, State, Resource>
    for WallDetector<Manager, Detector, N>
where
    Detector: Construct<Config, State, Resource>,
    Config: AsRef<WallDetectorConfig>,
    Resource: AsRef<Rc<Manager>>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let detector = Detector::construct(config, state, resource);
        let wall_manager = Rc::clone(resource.as_ref());
        let config = config.as_ref();
        let converter = PoseConverterBuilder::new()
            .square_width(config.square_width)
            .wall_width(config.wall_width)
            .ignore_radius_from_pillar(config.ignore_radius_from_pillar)
            .ignore_length_from_wall(config.ignore_length_from_wall)
            .build()
            .expect("Should never panic");
        Self::new(wall_manager, detector, converter)
    }
}

impl<Manager, Detector, State, Resource, const N: usize> Deconstruct<State, Resource>
    for WallDetector<Manager, Detector, N>
where
    Resource: Merge + From<Rc<Manager>>,
    Detector: Deconstruct<State, Resource>,
{
    fn deconstruct(self) -> (State, Resource) {
        let (manager, detector) = self.release();
        let (state, resource) = detector.deconstruct();
        (state, resource.merge(manager.into()))
    }
}

const OBSTACLE_SIZE_UPPER_BOUND: usize = 6;

impl<Manager, Detector, State, const N: usize> IWallDetector<State>
    for WallDetector<Manager, Detector, N>
where
    Manager: WallProbabilityManager<Wall<N>>,
    Detector: ObstacleDetector<State, Obstacle = Obstacle>,
{
    type Info = CorrectInfo;
    type Infos = Vec<CorrectInfo, OBSTACLE_SIZE_UPPER_BOUND>;

    fn detect_and_update(&mut self, state: &State) -> Self::Infos {
        use uom::si::ratio::ratio;

        let obstacles = self.detector.detect(state);

        let mut infos = ForcedVec::new();
        for obstacle in obstacles {
            if let Ok(wall_info) = self.converter.convert(&obstacle.source) {
                if let Ok(existence) = self.manager.try_existence_probability(&wall_info.wall) {
                    if existence.is_zero() {
                        continue;
                    } else if existence.is_one() {
                        let diff_from_expected =
                            obstacle.distance.mean - wall_info.existing_distance;
                        infos.push(CorrectInfo {
                            obstacle,
                            diff_from_expected,
                        });
                        continue;
                    }

                    let exist_val = {
                        let tmp = ((wall_info.existing_distance - obstacle.distance.mean)
                            / obstacle.distance.standard_deviation)
                            .get::<ratio>();
                        -tmp * tmp / 2.0
                    };
                    let not_exist_val = {
                        let tmp = ((wall_info.not_existing_distance - obstacle.distance.mean)
                            / obstacle.distance.standard_deviation)
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
                        Probability::new(exist_val / (exist_val + not_exist_val)).unwrap_or_else(
                            |err| unreachable!("Should never be out of bound: {:?}", err),
                        )
                    };
                    let _ = self.manager.try_update(&wall_info.wall, &existence);
                    //ignore if cannot update
                }
            }
        }
        infos.into()
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
        let pillars = if rot < 0.125 || rot > 0.875 {
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

        let create_error = || {
            Err(ConversionError::OutOfBound(OutOfBoundError {
                pose: pose.clone(),
            }))
        };

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

pub struct WallDetectorBuilder<Manager, Detector> {
    wall_manager: Option<Rc<Manager>>,
    obstacle_detector: Option<Detector>,
    square_width: Option<Length>,
    wall_width: Option<Length>,
    ignore_radius_from_pillar: Option<Length>,
    ignore_length_from_wall: Option<Length>,
}

impl<Manager, Detector> WallDetectorBuilder<Manager, Detector> {
    pub fn new() -> Self {
        Self {
            wall_manager: None,
            obstacle_detector: None,
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
        /// **Required**,
        /// Sets an implementation of [ObstacleDetector](ObstacleDetector).
        obstacle_detector: Detector
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

    pub fn build<const N: usize>(&mut self) -> BuilderResult<WallDetector<Manager, Detector, N>> {
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
            ok_or(self.obstacle_detector.take(), "obstacle_detector")?,
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
