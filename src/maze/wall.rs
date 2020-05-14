use core::marker::PhantomData;

use typenum::{PowerOfTwo, Unsigned};

use super::{Location, Position};

#[derive(Clone, Copy)]
pub enum WallDirection {
    Up,
    Right,
}

#[derive(Clone)]
pub struct WallPosition<N> {
    x: u16,
    y: u16,
    z: WallDirection,
    _size: PhantomData<fn() -> N>,
}

impl<N> WallPosition<N>
where
    N: Unsigned + PowerOfTwo,
{
    pub fn new(x: u16, y: u16, z: WallDirection) -> Self {
        Self {
            x,
            y,
            z,
            _size: PhantomData,
        }
    }

    #[inline]
    pub fn max() -> u16 {
        N::U16 - 1
    }

    #[inline]
    fn y_offset() -> u32 {
        N::USIZE.trailing_zeros()
    }

    #[inline]
    fn z_offset() -> u32 {
        2 * N::USIZE.trailing_zeros()
    }

    pub fn as_index(self) -> usize {
        self.x as usize
            | (self.y as usize) << Self::y_offset()
            | (self.z as usize) << Self::z_offset()
    }

    pub fn from_position(position: Position<N>) -> Option<Self> {
        use Location::*;
        let x = position.x() / 2;
        let y = position.y() / 2;
        if x < 0 || y < 0 {
            return None;
        }
        let z = match position.location() {
            Cell => return None,
            HorizontalBound => WallDirection::Up,
            VerticalBound => WallDirection::Right,
        };
        Some(Self::new(x as u16, y as u16, z))
    }
}
