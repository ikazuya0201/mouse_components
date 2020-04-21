use super::{MAX_H, MAX_W};
use crate::direction::AbsoluteDirection;

pub enum Location {
    Cell,
    VerticalBound,
    HorizontalBound,
}

#[derive(Clone, Copy)]
pub struct Node(u16);

impl Node {
    pub fn new(x: u16, y: u16, direction: AbsoluteDirection) -> Self {
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
    pub fn x(&self) -> u16 {
        self.0 & Self::x_max()
    }

    #[inline]
    pub fn y(&self) -> u16 {
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

    pub fn direction(&self) -> AbsoluteDirection {
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

    pub fn location(&self) -> Location {
        use Location::*;
        if self.x_is_even() {
            if self.y_is_even() {
                Cell
            } else {
                HorizontalBound
            }
        } else {
            if self.y_is_even() {
                VerticalBound
            } else {
                unreachable!()
            }
        }
    }

    pub fn next_straight(&self) -> Option<Self> {
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
