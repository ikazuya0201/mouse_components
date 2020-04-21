use core::marker::PhantomData;

use typenum::{PowerOfTwo, Unsigned};

use crate::direction::AbsoluteDirection;

pub enum Location {
    Cell,
    VerticalBound,
    HorizontalBound,
}

#[derive(Clone, Copy)]
pub struct Node<H, W> {
    raw: u16,
    _height: PhantomData<fn() -> H>,
    _width: PhantomData<fn() -> W>,
}

impl<H, W> Node<H, W>
where
    H: Unsigned + PowerOfTwo,
    W: Unsigned + PowerOfTwo,
{
    pub fn new(x: u16, y: u16, direction: AbsoluteDirection) -> Self {
        use AbsoluteDirection::*;
        debug_assert!(x <= Self::x_max());
        debug_assert!(y <= Self::y_max());
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
        Self {
            raw: x | (y << Self::y_offset()) | (direction << Self::direction_offset()),
            _height: PhantomData,
            _width: PhantomData,
        }
    }

    #[inline]
    fn x_min() -> u16 {
        0
    }

    #[inline]
    fn y_min() -> u16 {
        0
    }

    #[inline]
    fn x_max() -> u16 {
        W::U16 * 2 - 1
    }

    #[inline]
    fn y_max() -> u16 {
        H::U16 * 2 - 1
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

    #[inline]
    fn y_offset() -> u32 {
        (W::USIZE * 2).trailing_zeros()
    }

    #[inline]
    fn direction_offset() -> u32 {
        (W::USIZE * 2).trailing_zeros() + (H::USIZE * 2).trailing_zeros()
    }

    #[inline]
    pub fn x(&self) -> u16 {
        self.raw & Self::x_max()
    }

    #[inline]
    pub fn y(&self) -> u16 {
        (self.raw >> Self::y_offset()) & Self::y_max()
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
                match self.raw >> Self::direction_offset() {
                    0 => North,
                    1 => East,
                    2 => South,
                    3 => West,
                    _ => unreachable!(),
                }
            } else {
                match self.raw >> Self::direction_offset() {
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
                match self.raw >> Self::direction_offset() {
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

impl<H, W> Into<usize> for Node<H, W> {
    fn into(self) -> usize {
        self.raw.into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use typenum::consts::*;
    use AbsoluteDirection::*;

    #[test]
    fn test_base_methods() {
        let test_data = vec![
            (0u16, 0u16, North),
            (0, 0, East),
            (0, 0, South),
            (0, 0, West),
            (0, 1, North),
            (0, 1, NorthEast),
            (0, 1, SouthEast),
            (0, 1, South),
            (0, 1, SouthWest),
            (0, 1, NorthWest),
            (1, 0, NorthEast),
            (1, 0, East),
            (1, 0, SouthEast),
            (1, 0, SouthWest),
            (1, 0, West),
            (1, 0, NorthWest),
            (4, 4, South),
            (5, 4, West),
            (4, 5, North),
            (14, 14, North),
            (15, 14, East),
            (14, 15, South),
            (30, 30, North),
            (31, 30, East),
            (30, 31, South),
        ];

        for (x, y, direction) in test_data {
            let node = Node::<U16, U16>::new(x, y, direction);
            assert_eq!(node.x(), x);
            assert_eq!(node.y(), y);
            assert_eq!(node.direction(), direction);
        }
    }

    #[test]
    fn test_different_height_and_width() {
        let test_data = vec![(0u16, 0u16, North), (11, 14, East), (8, 7, North)];

        for (x, y, direction) in test_data {
            let node = Node::<U16, U8>::new(x, y, direction);
            println!("{} {}", x, y);
            assert_eq!(node.x(), x);
            assert_eq!(node.y(), y);
            assert_eq!(node.direction(), direction);
        }
    }

    #[test]
    #[should_panic]
    fn test_x_unreachable() {
        Node::<U16, U16>::new(32, 31, NorthEast);
    }

    #[test]
    #[should_panic]
    fn test_y_unreachable() {
        Node::<U16, U16>::new(31, 32, NorthEast);
    }

    #[test]
    #[should_panic]
    fn test_direction_unreachable_in_cell() {
        Node::<U16, U16>::new(0, 0, NorthEast);
    }

    #[test]
    #[should_panic]
    fn test_direction_unreachable_on_horizontal_bound() {
        Node::<U16, U16>::new(0, 1, East);
    }

    #[test]
    #[should_panic]
    fn test_direction_unreachable_on_vertical_bound() {
        Node::<U16, U16>::new(1, 0, North);
    }
}
