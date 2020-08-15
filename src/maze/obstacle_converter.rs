use core::marker::PhantomData;

use generic_array::GenericArray;
use quantities::{Distance, Quantity};
use typenum::{consts::*, PowerOfTwo, Unsigned};

use super::{WallDirection, WallPosition};
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

#[derive(Clone, Copy)]
enum Axis {
    X(Distance),
    Y(Distance),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutOfBoundError;

pub struct ObstacleConverter<N> {
    _phantom: PhantomData<fn() -> N>,
}

impl<N> ObstacleConverter<N>
where
    N: Unsigned + PowerOfTwo,
{
    const I_SQUARE_WIDTH: i32 = 9; //TODO: use configurable value
    const SQUARE_WIDTH: Distance = Distance::from_meters(0.09);
    const WALL_WIDTH_HALF: Distance = Distance::from_meters(0.003);

    pub fn new() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }

    //(remainder, quotient)
    fn remquof_with_width(val: Distance) -> (Distance, i32) {
        let quo = (val.as_meters() * 1000.0) as i32 / Self::I_SQUARE_WIDTH;
        let rem = val - Distance::from_meters((quo * Self::I_SQUARE_WIDTH) as f32 * 0.001);
        (rem, quo)
    }

    pub fn convert(&self, pose: Pose) -> Result<WallInfo<N>, OutOfBoundError> {
        let (x_rem, x_quo) = Self::remquof_with_width(pose.x);
        let (y_rem, y_quo) = Self::remquof_with_width(pose.y);
        let rot = pose.theta.as_bounded_rotation();

        let p1 = Self::SQUARE_WIDTH - Self::WALL_WIDTH_HALF;
        let p2 = p1 + Self::SQUARE_WIDTH;
        let n1 = Self::WALL_WIDTH_HALF;
        let n2 = n1 - Self::SQUARE_WIDTH;

        let axes = if rot < 0.25 {
            [Axis::X(p1), Axis::X(p2), Axis::Y(p1), Axis::Y(p2)]
        } else if rot < 0.5 {
            [Axis::X(n1), Axis::X(n2), Axis::Y(p1), Axis::Y(p2)]
        } else if rot < 0.75 {
            [Axis::X(n1), Axis::X(n2), Axis::Y(n1), Axis::Y(n2)]
        } else {
            [Axis::X(p1), Axis::X(p2), Axis::Y(n1), Axis::Y(n2)]
        };

        let (sin_th, cos_th) = libm::sincosf(pose.theta.as_radian());

        let mut axes_distance = axes
            .iter()
            .map(|&axis| {
                (
                    axis,
                    Total(match axis {
                        Axis::X(x) => (x - x_rem) / cos_th,
                        Axis::Y(y) => (y - y_rem) / sin_th,
                    }),
                )
            })
            .collect::<GenericArray<_, U4>>();

        axes_distance.sort_unstable_by_key(|e| e.1);

        let wall_position = match axes_distance[0].0 {
            Axis::X(x) => {
                let (x, y) = if x.is_positive() {
                    (x_quo, y_quo)
                } else {
                    (x_quo - 1, y_quo)
                };
                if x < 0 || y < 0 {
                    return Err(OutOfBoundError);
                }
                WallPosition::new(x as u16, y as u16, WallDirection::Right)
            }
            Axis::Y(y) => {
                let (x, y) = if y.is_positive() {
                    (x_quo, y_quo)
                } else {
                    (x_quo, y_quo - 1)
                };
                if x < 0 || y < 0 {
                    return Err(OutOfBoundError);
                }
                WallPosition::new(x as u16, y as u16, WallDirection::Up)
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
}
