use core::ops::Mul;

use generic_array::GenericArray;
use heapless::{consts::*, ArrayLength, Vec};
use static_assertions::const_assert;

use crate::direction::AbsoluteDirection;
use crate::operator::{
    CheckableGraph, DirectionInstructor, DirectionalGraph, Graph, GraphTranslator,
};

const MAX_H: usize = 32;
const MAX_W: usize = 32;

const_assert!(MAX_H.is_power_of_two());
const_assert!(MAX_W.is_power_of_two());

pub struct Node(u16);

impl Node {
    fn new(x: u16, y: u16, direction: AbsoluteDirection) -> Self {
        use AbsoluteDirection::*;
        let direction = if x & 1 == 0 {
            if y & 1 == 0 {
                match direction {
                    North => 0,
                    East => 1,
                    South => 2,
                    West => 3,
                    _ => unreachable!(),
                }
            } else {
                match direction {
                    North => 0,
                    NorthEast => 1,
                    SouthEast => 2,
                    South => 3,
                    SouthWest => 4,
                    NorthWest => 5,
                    _ => unreachable!(),
                }
            }
        } else {
            if y & 1 == 0 {
                match direction {
                    NorthEast => 0,
                    East => 1,
                    SouthEast => 2,
                    SouthWest => 3,
                    West => 4,
                    NorthWest => 5,
                    _ => unreachable!(),
                }
            } else {
                unreachable!()
            }
        };
        Self(x | (y << Self::y_offset()) | (direction << Self::direction_offset()))
    }

    const fn x_min() -> u16 {
        0
    }

    const fn y_min() -> u16 {
        0
    }

    const fn x_max() -> u16 {
        MAX_W as u16 * 2 - 1
    }

    const fn y_max() -> u16 {
        MAX_H as u16 * 2 - 1
    }

    #[inline]
    fn x_is_min(&self) -> bool {
        self.x() == Self::x_min()
    }

    #[inline]
    fn y_is_min(&self) -> bool {
        self.y() == Self::y_min()
    }

    #[inline]
    fn x_is_max(&self) -> bool {
        self.x() == Self::x_max()
    }

    #[inline]
    fn y_is_max(&self) -> bool {
        self.y() == Self::y_max()
    }

    const fn y_offset() -> u32 {
        (MAX_W * 2).trailing_zeros()
    }

    const fn direction_offset() -> u32 {
        (MAX_W * 2).trailing_zeros() + (MAX_H * 2).trailing_zeros()
    }

    #[inline]
    fn x(&self) -> u16 {
        self.0 & Self::x_max()
    }

    #[inline]
    fn y(&self) -> u16 {
        (self.0 >> Self::y_offset()) & Self::y_max()
    }

    #[inline]
    fn x_is_even(&self) -> bool {
        self.x() & 1 == 0
    }

    #[inline]
    fn y_is_even(&self) -> bool {
        self.y() & 1 == 0
    }

    fn direction(&self) -> AbsoluteDirection {
        use AbsoluteDirection::*;
        if self.x_is_even() {
            if self.y_is_even() {
                match self.0 >> Self::direction_offset() {
                    0 => North,
                    1 => East,
                    2 => South,
                    3 => West,
                    _ => unreachable!(),
                }
            } else {
                match self.0 >> Self::direction_offset() {
                    0 => North,
                    1 => NorthEast,
                    2 => SouthEast,
                    3 => South,
                    4 => SouthWest,
                    5 => NorthWest,
                    _ => unreachable!(),
                }
            }
        } else {
            if self.y_is_even() {
                match self.0 >> Self::direction_offset() {
                    0 => NorthEast,
                    1 => East,
                    2 => SouthEast,
                    3 => SouthWest,
                    4 => West,
                    5 => NorthWest,
                    _ => unreachable!(),
                }
            } else {
                unreachable!()
            }
        }
    }

    fn on_wall(&self) -> bool {
        self.on_vertical_wall() || self.on_horizontal_wall()
    }

    fn on_vertical_wall(&self) -> bool {
        self.x() & 1 == 1 && self.y() & 1 == 0
    }

    fn on_horizontal_wall(&self) -> bool {
        self.x() & 1 == 0 && self.y() & 1 == 1
    }

    fn in_cell(&self) -> bool {
        self.x() & 1 == 0 && self.y() & 0 == 0
    }

    fn next(&self) -> Option<Self> {
        use AbsoluteDirection::*;
        let direction = self.direction();
        match direction {
            North => {
                if self.y_is_max() {
                    None
                } else {
                    Some(Self::new(self.x(), self.y() + 1, direction))
                }
            }
            NorthEast => {
                if self.x_is_max() || self.y_is_max() {
                    None
                } else {
                    Some(Self::new(self.x() + 1, self.y() + 1, direction))
                }
            }
            East => {
                if self.x_is_max() {
                    None
                } else {
                    Some(Self::new(self.x() + 1, self.y(), direction))
                }
            }
            SouthEast => {
                if self.x_is_max() || self.y_is_min() {
                    None
                } else {
                    Some(Self::new(self.x() + 1, self.y() - 1, direction))
                }
            }
            South => {
                if self.y_is_min() {
                    None
                } else {
                    Some(Self::new(self.x(), self.y() - 1, direction))
                }
            }
            SouthWest => {
                if self.x_is_min() || self.y_is_min() {
                    None
                } else {
                    Some(Self::new(self.x() - 1, self.y() - 1, direction))
                }
            }
            West => {
                if self.x_is_min() {
                    None
                } else {
                    Some(Self::new(self.x() - 1, self.y(), direction))
                }
            }
            NorthWest => {
                if self.x_is_min() || self.y_is_max() {
                    None
                } else {
                    Some(Self::new(self.x() - 1, self.y() + 1, direction))
                }
            }
        }
    }
}

impl Into<usize> for Node {
    fn into(self) -> usize {
        self.0.into()
    }
}

pub struct Maze {
    h: usize,
    w: usize,
    is_checked: [bool; MAX_H * MAX_W * 2],
    is_wall: [bool; MAX_H * MAX_W * 2],
}

impl Maze {
    pub fn new(h: usize, w: usize) -> Self {
        Self {
            h,
            w,
            is_checked: [false; MAX_H * MAX_W * 2],
            is_wall: [false; MAX_H * MAX_W * 2],
        }
    }

    fn next(&self, node: Node) -> Option<Node> {
        None
    }
}

impl Graph<Node, u16> for Maze {
    type Edges = Vec<(Node, u16), U4096>;

    fn successors(&self, node: Node) -> Self::Edges {
        let mut succs = Vec::new();
        succs
    }

    fn predecessors(&self, node: Node) -> Self::Edges {
        let mut preds = Vec::new();
        preds
    }
}
