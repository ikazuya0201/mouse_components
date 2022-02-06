use core::marker::PhantomData;

use heapless::Vec;
#[allow(unused_imports)]
use micromath::F32Ext;
use uom::si::{
    angle::revolution,
    f32::{Angle, Length},
    length::meter,
};

use crate::solver::{Coordinate, WallState};
use crate::WIDTH;

struct WallInfo<const W: u8> {
    pub coord: Coordinate<W>,
    pub existing_distance: Length,
    pub not_existing_distance: Length,
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Pose {
    pub x: Length,
    pub y: Length,
    pub theta: Angle,
}

#[derive(Debug)]
pub struct WallDetector<const W: u8> {
    converter: PoseConverter<W>,
    wall_existence_array: [[[Probability; 2]; WIDTH]; WIDTH],
}

#[derive(Debug)]
pub struct Normal<T> {
    pub mean: T,
    pub stddev: T,
}

#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
struct Probability(f32);

impl Probability {
    fn new(val: f32) -> Option<Self> {
        if val.is_nan() || val.is_sign_negative() || val > 1.0 {
            None
        } else {
            Some(Self(val))
        }
    }

    const fn one() -> Self {
        Probability(1.0)
    }

    const fn zero() -> Self {
        Probability(0.0)
    }

    const fn mid() -> Self {
        Probability(0.5)
    }

    fn reverse(&self) -> Self {
        Probability(1.0 - self.0)
    }

    fn is_one(&self) -> bool {
        (self.0 - 1.0).abs() < core::f32::EPSILON
    }

    fn is_zero(&self) -> bool {
        self.0.abs() < core::f32::EPSILON
    }
}

impl<const W: u8> Default for WallDetector<W> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const W: u8> WallDetector<W> {
    pub fn new() -> Self {
        Self {
            converter: PoseConverter::default(),
            wall_existence_array: [[[Probability::mid(); 2]; WIDTH]; WIDTH],
        }
    }

    fn wall_existence(&self, coord: &Coordinate<W>) -> &Probability {
        &self.wall_existence_array[coord.x as usize][coord.y as usize][coord.is_top as usize]
    }

    fn wall_existence_mut(&mut self, coord: &Coordinate<W>) -> &mut Probability {
        &mut self.wall_existence_array[coord.x as usize][coord.y as usize][coord.is_top as usize]
    }

    pub fn detect_and_update(
        &mut self,
        distance: Normal<Length>,
        pose: Pose,
    ) -> Option<(Coordinate<W>, WallState)> {
        use uom::si::ratio::ratio;

        let wall_info = self.converter.convert(&pose)?;

        let existence = *self.wall_existence(&wall_info.coord);

        if existence.is_zero() | existence.is_one() {
            return None;
        }

        let exist_val = {
            let tmp =
                ((wall_info.existing_distance - distance.mean) / distance.stddev).get::<ratio>();
            -tmp * tmp / 2.0
        };
        let not_exist_val = {
            let tmp = ((wall_info.not_existing_distance - distance.mean) / distance.stddev)
                .get::<ratio>();
            -tmp * tmp / 2.0
        };

        assert!(!exist_val.is_nan());
        assert!(!not_exist_val.is_nan());

        let min = if exist_val < not_exist_val {
            exist_val
        } else {
            not_exist_val
        };
        let exist_val = (exist_val - min).exp() * existence.0;
        let not_exist_val = (not_exist_val - min).exp() * existence.reverse().0;

        let existence = if exist_val.is_infinite() {
            Probability::one()
        } else if not_exist_val.is_infinite() {
            Probability::zero()
        } else {
            Probability::new(exist_val / (exist_val + not_exist_val))
                .expect("Should never fail: this probability must be in the range of [0, 1].")
        };

        *self.wall_existence_mut(&wall_info.coord) = existence;

        let state = if existence < WALL_EXISTENCE_TH {
            WallState::Checked { exists: false }
        } else if existence > WALL_EXISTENCE_TH.reverse() {
            WallState::Checked { exists: true }
        } else {
            WallState::Unchecked
        };

        Some((wall_info.coord, state))
    }
}

const WALL_EXISTENCE_TH: Probability = Probability(0.1);

#[derive(Clone, PartialEq, Debug)]
struct PoseConverter<const W: u8> {
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

const DEFAULT_SQUARE_WIDTH: Length = Length {
    dimension: PhantomData,
    units: PhantomData,
    value: 0.09,
};
const DEFAULT_WALL_WIDTH: Length = Length {
    dimension: PhantomData,
    units: PhantomData,
    value: 0.006,
};
const DEFAULT_IGNORE_RADIUS: Length = Length {
    dimension: PhantomData,
    units: PhantomData,
    value: 0.01,
};
const DEFAULT_IGNORE_LENGTH: Length = Length {
    value: 0.008,
    dimension: PhantomData,
    units: PhantomData,
};

impl<const W: u8> Default for PoseConverter<W> {
    fn default() -> Self {
        Self::new(
            DEFAULT_SQUARE_WIDTH,
            DEFAULT_WALL_WIDTH,
            DEFAULT_IGNORE_RADIUS,
            DEFAULT_IGNORE_LENGTH,
        )
    }
}

impl<const W: u8> PoseConverter<W> {
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

impl<const W: u8> PoseConverter<W> {
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
            .map(|pillar| (sin_th * (pose.x - pillar.0) - cos_th * (pose.y - pillar.1)).abs())
            .fold(Length::new::<meter>(core::f32::INFINITY), |m, v| m.min(v));

        mind < self.ignore_radius_from_pillar
    }
}

impl<const W: u8> PoseConverter<W> {
    fn convert(&self, pose: &Pose) -> Option<WallInfo<W>> {
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
            return None;
        }

        if x_rem <= self.n1 || x_rem >= self.p1 || y_rem <= self.n1 || y_rem >= self.p1 {
            return None;
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

        axes_distance.sort_unstable_by(|a, b| {
            a.1.partial_cmp(&b.1)
                .expect("Should never fail: all distances must not be `NAN`.")
        });

        let wall = match axes_distance[0].0 {
            Right => {
                if x_quo < 0 || y_quo < 0 {
                    return None;
                }
                Coordinate::new(x_quo as u8, y_quo as u8, false)
            }
            Left => {
                if x_quo < 1 || y_quo < 0 {
                    return None;
                }
                Coordinate::new((x_quo - 1) as u8, y_quo as u8, false)
            }
            Top => {
                if x_quo < 0 || y_quo < 0 {
                    return None;
                }
                Coordinate::new(x_quo as u8, y_quo as u8, true)
            }
            Bottom => {
                if x_quo < 0 || y_quo < 1 {
                    return None;
                }
                Coordinate::new(x_quo as u8, (y_quo - 1) as u8, true)
            }
        }?;

        let existing_distance = axes_distance[0].1; //the nearest one
        let not_existing_distance = axes_distance[1].1; //the 2nd nearest one

        if existing_distance <= self.ignore_length_from_wall {
            return None;
        }

        Some(WallInfo {
            coord: wall,
            existing_distance,
            not_existing_distance,
        })
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use uom::si::angle::degree;
    use uom::si::f32::Angle;

    use super::*;

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
                    coord: Coordinate::<$size>::new(expected.0, expected.1, expected.2).unwrap(),
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
                assert_eq!(info.coord, expected.coord);
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
                assert!(converter.convert(&input).is_none());
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
