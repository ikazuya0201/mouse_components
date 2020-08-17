use generic_array::GenericArray;
use quantities::{Angle, Distance, Quantity};
use typenum::{consts::*, PowerOfTwo, Unsigned};

use super::{AbsoluteDirection, SearchNodeId, WallDirection, WallPosition};
use crate::agent::Pose;
use crate::utils::total::Total;

#[derive(Debug, Clone, PartialEq)]
pub struct WallInfo<N>
where
    N: Unsigned + PowerOfTwo,
{
    pub position: WallPosition<N>,
    pub existing_distance: Distance,
    pub not_existing_distance: Distance,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutOfBoundError;

pub struct PoseConverter {
    i_square_width: i32, //[mm]
    square_width_half: Distance,
    p1: Distance,
    p2: Distance,
    n1: Distance,
    n2: Distance,
}

impl PoseConverter {
    pub fn new(square_width: Distance, wall_width: Distance) -> Self {
        let p1 = square_width - wall_width / 2.0;
        let p2 = p1 + square_width;
        let n1 = wall_width / 2.0;
        let n2 = n1 - square_width;
        Self {
            i_square_width: (square_width.as_meters() * 1000.0) as i32,
            square_width_half: square_width / 2.0,
            p1,
            p2,
            n1,
            n2,
        }
    }

    //(remainder, quotient)
    fn remquof_with_width(&self, val: Distance) -> (Distance, i32) {
        let quo = (val.as_meters() * 1000.0) as i32 / self.i_square_width;
        let rem = val - Distance::from_meters((quo * self.i_square_width) as f32 * 0.001);
        (rem, quo)
    }

    pub fn convert<N>(&self, pose: Pose) -> Result<WallInfo<N>, OutOfBoundError>
    where
        N: Unsigned + PowerOfTwo,
    {
        #[derive(Clone, Copy, Debug)]
        enum Axis {
            X(Distance),
            Y(Distance),
        }

        #[derive(Clone, Copy, Debug)]
        enum Direction {
            Right,
            Left,
            Top,
            Bottom,
        }

        use Direction::*;

        let (x_rem, x_quo) = self.remquof_with_width(pose.x);
        let (y_rem, y_quo) = self.remquof_with_width(pose.y);

        if x_rem <= self.n1 || x_rem >= self.p1 || y_rem <= self.n1 || y_rem >= self.p1 {
            return Err(OutOfBoundError);
        }

        let rot = pose.theta.as_bounded_rotation();

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

        let (sin_th, cos_th) = libm::sincosf(pose.theta.as_radian());

        let mut axes_distance = axes
            .iter()
            .map(|&(axis, direction)| {
                let dist = match axis {
                    Axis::X(x) => (x - x_rem) / cos_th,
                    Axis::Y(y) => (y - y_rem) / sin_th,
                };
                //assign infinity to invalid values
                let dist = if dist.is_negative() || dist.as_meters().is_nan() {
                    Distance::from_meters(core::f32::INFINITY)
                } else {
                    dist
                };
                (direction, Total(dist))
            })
            .collect::<GenericArray<_, U4>>();

        axes_distance.sort_unstable_by_key(|e| e.1);

        let wall_position = match axes_distance[0].0 {
            Right => {
                if x_quo < 0 || y_quo < 0 {
                    return Err(OutOfBoundError);
                }
                WallPosition::new(x_quo as u16, y_quo as u16, WallDirection::Right)
            }
            Left => {
                if x_quo < 1 || y_quo < 0 {
                    return Err(OutOfBoundError);
                }
                WallPosition::new((x_quo - 1) as u16, y_quo as u16, WallDirection::Right)
            }
            Top => {
                if x_quo < 0 || y_quo < 0 {
                    return Err(OutOfBoundError);
                }
                WallPosition::new(x_quo as u16, y_quo as u16, WallDirection::Up)
            }
            Bottom => {
                if x_quo < 0 || y_quo < 1 {
                    return Err(OutOfBoundError);
                }
                WallPosition::new(x_quo as u16, (y_quo - 1) as u16, WallDirection::Up)
            }
        };

        if let Some(wall_position) = wall_position {
            Ok(WallInfo {
                position: wall_position,
                existing_distance: (axes_distance[0].1).0,
                not_existing_distance: (axes_distance[1].1).0,
            })
        } else {
            Err(OutOfBoundError)
        }
    }

    pub fn convert_node<N>(&self, node: SearchNodeId<N>) -> Pose
    where
        N: Unsigned + PowerOfTwo,
    {
        use AbsoluteDirection::*;

        let node = node.as_node();
        let x = node.x();
        let y = node.y();
        let direction = node.direction();
        Pose {
            x: (x + 1) as f32 * self.square_width_half,
            y: (y + 1) as f32 * self.square_width_half,
            theta: match direction {
                East => Angle::from_degree(0.0),
                North => Angle::from_degree(90.0),
                West => Angle::from_degree(180.0),
                South => Angle::from_degree(270.0),
                _ => unreachable!(),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;
    use quantities::Angle;
    use WallDirection::*;

    macro_rules! convert_ok_tests {
        ($($name:ident: ($size:ty, $value: expr),)*) => {
            $(
                #[test]
                fn $name() {
                    let (input, expected) = $value;
                    let input = Pose{
                        x: Distance::from_meters(input.0),
                        y: Distance::from_meters(input.1),
                        theta: Angle::from_degree(input.2),
                    };
                    let expected = WallInfo::<$size>{
                        position: WallPosition::new(expected.0, expected.1, expected.2).unwrap(),
                        existing_distance: Distance::from_meters(expected.3),
                        not_existing_distance: Distance::from_meters(expected.4),
                    };

                    let converter = PoseConverter::new(Distance::from_meters(0.09), Distance::from_meters(0.006));
                    let info = converter.convert::<$size>(input).unwrap();
                    assert_eq!(info.position, expected.position);
                    assert_relative_eq!(info.existing_distance.as_meters(), expected.existing_distance.as_meters());
                    assert_relative_eq!(info.not_existing_distance.as_meters(), expected.not_existing_distance.as_meters());
                }
            )*
        }
    }

    macro_rules! convert_err_tests {
        ($($name:ident: ($size:ty, $value: expr),)*) => {
            $(
                #[test]
                fn $name() {
                    let input = $value;
                    let input = Pose{
                        x: Distance::from_meters(input.0),
                        y: Distance::from_meters(input.1),
                        theta: Angle::from_degree(input.2),
                    };

                    let converter = PoseConverter::new(Distance::from_meters(0.09), Distance::from_meters(0.006));
                    assert!(converter.convert::<$size>(input).is_err());
                }
            )*
        }
    }

    convert_ok_tests! {
        convert_ok_test1: (U4, (
            (0.045, 0.045, 0.0),
            (0, 0, Right, 0.042, 0.132),
        )),
        convert_ok_test2: (U4, (
            (0.077, 0.045, 45.0),
            (0, 0, Right, 2.0f32.sqrt() * 0.01, 2.0f32.sqrt() * 0.042),
        )),
        convert_ok_test3: (U4, (
            (0.135, 0.045, 180.0),
            (0, 0, Right, 0.042, 0.132),
        )),
        convert_ok_test4: (U4, (
            (0.045, 0.135, 270.0),
            (0, 0, Up, 0.042, 0.132),
        )),
        convert_ok_test5: (U4, (
            (0.135, 0.135, 90.0),
            (1, 1, Up, 0.042, 0.132),
        )),
        convert_ok_test6: (U4, (
            (0.135, 0.135, 180.0),
            (0, 1, Right, 0.042, 0.132),
        )),
    }

    convert_err_tests! {
        convert_err_test1: (U4, (0.0, 0.0, 0.0)),
        convert_err_test2: (U4, (0.405, 0.045, 0.0)),
        convert_err_test3: (U4, (-0.045, 0.045, 0.0)),
    }
}
