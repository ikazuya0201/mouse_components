use core::marker::PhantomData;

use typenum::{PowerOfTwo, Unsigned};

use super::{Location, Position};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WallDirection {
    Up = 1,
    Right = 0,
}

#[derive(Debug, Clone, PartialEq, Eq)]
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
    pub fn new(x: u16, y: u16, z: WallDirection) -> Option<Self> {
        if x > Self::max() || y > Self::max() {
            None
        } else {
            Some(Self::new_unchecked(x, y, z))
        }
    }

    fn new_unchecked(x: u16, y: u16, z: WallDirection) -> Self {
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
        N::USIZE.trailing_zeros() + 1
    }

    #[inline]
    fn x_offset() -> u32 {
        1
    }

    pub fn as_index(self) -> usize {
        self.z as usize
            | (self.x as usize) << Self::x_offset()
            | (self.y as usize) << Self::y_offset()
    }

    pub fn from_position(position: Position<N>) -> Option<Self> {
        use Location::*;
        if position.x() < 0 || position.y() < 0 {
            return None;
        }
        let x = position.x() / 2;
        let y = position.y() / 2;
        let z = match position.location() {
            Cell => return None,
            HorizontalBound => WallDirection::Up,
            VerticalBound => WallDirection::Right,
        };
        Some(Self::new_unchecked(x as u16, y as u16, z))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use typenum::consts::*;

    macro_rules! from_position_tests {
        ($($name: ident: $value: expr,)*) => {
            $(
                #[test]
                fn $name() {
                    let (input, expected) = $value;
                    let position = Position::<U4>::new(input.0, input.1);
                    let expected = expected.map(|(x,y,dir)| WallPosition::<U4>::new(x,y,dir).unwrap());
                    assert_eq!(WallPosition::from_position(position), expected);
                }
            )*
        }
    }

    from_position_tests! {
        test_from_position1: ((2,-1), None),
        test_from_position2: ((1,0), Some((0,0,WallDirection::Right))),
    }
}
