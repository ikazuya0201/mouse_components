use core::{fmt, marker::PhantomData};

use heapless::Vec;
#[allow(unused_imports)]
use micromath::F32Ext;
use uom::si::{
    angle::{degree, revolution},
    f32::{Angle, Length},
    length::meter,
};

use crate::solver::{AbsoluteDirection, Coordinate, SearchState, WallState};
use crate::WIDTH;

pub struct WallInfo<const W: u8> {
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

impl Pose {
    pub fn from_search_state<const W: u8>(
        value: SearchState<W>,
        square_width: Length,
        front_offset: Length,
    ) -> Self {
        let (x, y) = (
            (2.0 * (value.coord.x as f32) + 1.0 + if value.coord.is_top { 0.0 } else { 1.0 })
                * square_width
                / 2.0,
            (2.0 * (value.coord.y as f32) + 1.0 + if value.coord.is_top { 1.0 } else { 0.0 })
                * square_width
                / 2.0,
        );
        let (x, y, theta) = match value.dir {
            AbsoluteDirection::North => (x, y + front_offset, 90.0),
            AbsoluteDirection::South => (x, y - front_offset, -90.0),
            AbsoluteDirection::East => (x + front_offset, y, 0.0),
            AbsoluteDirection::West => (x - front_offset, y, 180.0),
        };
        Pose {
            x,
            y,
            theta: Angle::new::<degree>(theta),
        }
    }
}

#[derive(Debug)]
pub struct WallDetector<const W: u8> {
    converter: PoseConverter<W>,
    wall_existence_array: [[[Probability; 2]; WIDTH]; WIDTH],
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

    pub fn with_walls(walls: &Walls<W>) -> Self {
        let mut detector = Self::new();
        for i in 0..W {
            for j in 0..W {
                for k in 0..2 {
                    let coord = Coordinate::<W>::new(i, j, k == 0).unwrap();
                    *detector.wall_existence_mut(&coord) = match walls.wall_state(&coord) {
                        WallState::Unchecked => Probability::mid(),
                        WallState::Checked { exists: true } => Probability::one(),
                        WallState::Checked { exists: false } => Probability::zero(),
                    };
                }
            }
        }
        detector
    }

    fn wall_existence(&self, coord: &Coordinate<W>) -> &Probability {
        &self.wall_existence_array[coord.x as usize][coord.y as usize][coord.is_top as usize]
    }

    fn wall_existence_mut(&mut self, coord: &Coordinate<W>) -> &mut Probability {
        &mut self.wall_existence_array[coord.x as usize][coord.y as usize][coord.is_top as usize]
    }

    pub fn detect_and_update(
        &mut self,
        &dist_mean: &Length,
        &dist_stddev: &Length,
        pose: &Pose,
    ) -> Option<(Coordinate<W>, WallState)> {
        use uom::si::ratio::ratio;

        let wall_info = self.converter.convert(pose)?;

        let existence = *self.wall_existence(&wall_info.coord);

        if existence.is_zero() | existence.is_one() {
            return None;
        }

        let exist_val = {
            let tmp = ((wall_info.existing_distance - dist_mean) / dist_stddev).get::<ratio>();
            -tmp * tmp / 2.0
        };
        let not_exist_val = {
            let tmp = ((wall_info.not_existing_distance - dist_mean) / dist_stddev).get::<ratio>();
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
pub struct PoseConverter<const W: u8> {
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
    pub fn convert(&self, pose: &Pose) -> Option<WallInfo<W>> {
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

const WALL_ARRAY_LEN: usize = WIDTH * WIDTH / 2;

#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Walls<const W: u8>([u8; WALL_ARRAY_LEN]);

impl<const W: u8> Default for Walls<W> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const W: u8> Walls<W> {
    pub fn new() -> Self {
        let mut walls = Self([0; WALL_ARRAY_LEN]);
        for i in 0..W {
            walls.update(
                &Coordinate {
                    x: W - 1,
                    y: i,
                    is_top: false,
                },
                &WallState::Checked { exists: true },
            );
            walls.update(
                &Coordinate {
                    x: i,
                    y: W - 1,
                    is_top: true,
                },
                &WallState::Checked { exists: true },
            );
        }
        walls.update(
            &Coordinate {
                x: 0,
                y: 0,
                is_top: false,
            },
            &WallState::Checked { exists: true },
        );
        walls.update(
            &Coordinate {
                x: 0,
                y: 0,
                is_top: true,
            },
            &WallState::Checked { exists: false },
        );
        walls
    }

    pub fn update(&mut self, coord: &Coordinate<W>, state: &WallState) {
        let bit = match state {
            WallState::Unchecked => 0,
            WallState::Checked { exists: false } => 1,
            WallState::Checked { exists: true } => 3,
        };
        let index = coord.as_index();
        let shift = (index & 3) << 1;
        (self.0)[index >> 2] &= !(3 << shift);
        (self.0)[index >> 2] |= bit << shift;
    }

    pub fn wall_state(&self, wall: &Coordinate<W>) -> WallState {
        let index = wall.as_index();
        let bit = ((self.0)[index >> 2] >> ((index & 3) << 1)) & 3;
        match bit {
            0 => WallState::Unchecked,
            1 => WallState::Checked { exists: false },
            3 => WallState::Checked { exists: true },
            _ => unreachable!(),
        }
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.0
    }
}

impl<'a, const W: u8> TryFrom<&'a [u8]> for Walls<W> {
    type Error = <[u8; WALL_ARRAY_LEN] as TryFrom<&'a [u8]>>::Error;

    fn try_from(value: &'a [u8]) -> Result<Self, Self::Error> {
        Ok(Self(value.try_into()?))
    }
}

#[derive(Debug, PartialEq, Eq)]
pub struct ParseWallsError {
    kind: ParseWallsErrorKind,
    line: usize,
    char_range: (usize, usize),
}

#[derive(Debug, PartialEq, Eq)]
enum ParseWallsErrorKind {
    One { expected: &'static str },
    Two { expected: [&'static str; 2] },
}

impl ParseWallsError {
    fn new(expected: &'static str, line: usize, char_range: (usize, usize)) -> Self {
        Self {
            kind: ParseWallsErrorKind::One { expected },
            line,
            char_range,
        }
    }

    fn new_two(expected: [&'static str; 2], line: usize, char_range: (usize, usize)) -> Self {
        Self {
            kind: ParseWallsErrorKind::Two { expected },
            line,
            char_range,
        }
    }
}

impl fmt::Display for ParseWallsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "line {}, character {} to {}, ",
            self.line + 1,
            self.char_range.0,
            self.char_range.1
        )?;
        match self.kind {
            ParseWallsErrorKind::One { expected } => write!(f, "here must be `{}`.", expected)?,
            ParseWallsErrorKind::Two { expected } => {
                write!(f, "here must be `{}` or `{}`.", expected[0], expected[1])?
            }
        }
        Ok(())
    }
}

impl<const W: u8> core::str::FromStr for Walls<W> {
    type Err = ParseWallsError;

    // TODO: validation
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut walls = Self::new();
        let mut update = |x, y, is_top, exists| {
            walls.update(&Coordinate { x, y, is_top }, &WallState::Checked { exists });
        };
        let check = |s: &str, expected, len, y, x| {
            if s.get(..len) == Some(expected) {
                Ok(())
            } else {
                Err(ParseWallsError::new(expected, y, (x, x + len)))
            }
        };

        for (y, mut s) in s.lines().enumerate().take(2 * W as usize + 1) {
            let y = y as u8;
            if y == 2 * W {
                check(s, "+", 1, y as usize, 0)?;
                s = &s[1..];
                for x in 0..W {
                    check(s, "---+", 4, y as usize, 4 * x as usize + 1)?;
                    s = &s[4..];
                }
                break;
            }
            for x in 0..W {
                if y & 1 == 0 {
                    if x == 0 {
                        check(s, "+", 1, y as usize, x as usize)?;
                        s = &s[1..];
                    }
                    match s.get(..4) {
                        Some("---+") => update(x, W - y / 2 - 1, true, true),
                        Some("   +") => update(x, W - y / 2 - 1, true, false),
                        _ => {
                            return Err(ParseWallsError::new_two(
                                ["---+", "   +"],
                                y as usize,
                                (4 * x as usize + 1, 4 * x as usize + 5),
                            ))
                        }
                    }
                    s = &s[4..];
                } else {
                    if x == 0 {
                        check(s, "|", 1, y as usize, x as usize)?;
                        s = &s[1..];
                    }
                    match s.get(..4) {
                        Some("   |") => update(x, W - y / 2 - 1, false, true),
                        Some("    ") => update(x, W - y / 2 - 1, false, false),
                        _ => {
                            return Err(ParseWallsError::new_two(
                                ["   |", "    "],
                                y as usize,
                                (4 * x as usize + 1, 4 * x as usize + 5),
                            ))
                        }
                    }
                    s = &s[4..];
                }
            }
        }
        Ok(walls)
    }
}

impl<const W: u8> fmt::Display for Walls<W> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let exists = |x, y, is_top| {
            matches!(
                self.wall_state(&Coordinate::new(x, y, is_top).unwrap()),
                WallState::Checked { exists: true }
            )
        };
        for y in (0..W).rev() {
            for x in 0..W {
                if exists(x, y, true) {
                    write!(f, "+---")?;
                } else {
                    write!(f, "+   ")?;
                }
            }
            writeln!(f, "+")?;

            write!(f, "|")?;
            for x in 0..W {
                if exists(x, y, false) {
                    write!(f, "   |")?;
                } else {
                    write!(f, "    ")?;
                }
            }
            writeln!(f)?;
        }
        for _ in 0..W {
            write!(f, "+---")?;
        }
        writeln!(f, "+")?;
        Ok(())
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

    #[test]
    fn test_empty_display() {
        let s = Walls::<4>::new().to_string();
        let expected = include_str!("../mazes/empty4.dat");
        assert_eq!(&s, expected);
    }

    #[test]
    fn test_simple_display() {
        let mut walls = Walls::<4>::new();
        let vec = vec![
            (0, 0, false),
            (0, 1, false),
            (0, 2, false),
            (1, 2, true),
            (2, 2, true),
            (2, 2, false),
            (1, 1, false),
            (2, 0, false),
            (2, 0, true),
        ];
        for (x, y, is_top) in vec {
            walls.update(
                &Coordinate { x, y, is_top },
                &WallState::Checked { exists: true },
            );
        }
        let s = walls.to_string();
        let expected = include_str!("../mazes/maze4_1.dat");
        assert_eq!(&s, expected);
    }

    #[test]
    fn test_parse() {
        let test_cases = vec![
            include_str!("../mazes/maze4_1.dat"),
            include_str!("../mazes/maze4_2.dat"),
            include_str!("../mazes/maze4_3.dat"),
        ];

        for s in test_cases {
            let s2 = s.parse::<Walls<4>>().unwrap().to_string();
            assert_eq!(&s2, s);
        }
    }

    #[test]
    fn test_parse_error() {
        let test_cases = vec![
            (
                include_str!("../mazes/invalid4_1.dat"),
                Err(ParseWallsError::new_two(["---+", "   +"], 0, (5, 9))),
            ),
            (
                include_str!("../mazes/invalid4_2.dat"),
                Err(ParseWallsError::new_two(["   |", "    "], 7, (5, 9))),
            ),
        ];

        for (input, expected) in test_cases {
            assert_eq!(input.parse::<Walls<4>>(), expected);
        }
    }
}
