use core::cell::RefCell;
use core::marker::PhantomData;
use core::ops::Mul;

use generic_array::{ArrayLength, GenericArray};
use typenum::{consts::U2, PowerOfTwo, Unsigned};

use crate::simple_maze::WallManager;
use crate::utils::{itertools::repeat_n, probability::Probability};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Wall<N> {
    x: u16,
    y: u16,
    top: bool,
    _maze_width: PhantomData<fn() -> N>,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct OutOfBoundError {
    x: u16,
    y: u16,
    top: bool,
}

impl<N> Wall<N> {
    pub fn x(&self) -> u16 {
        self.x
    }

    pub fn y(&self) -> u16 {
        self.y
    }

    pub fn is_top(&self) -> bool {
        self.top
    }

    #[inline]
    fn y_offset() -> usize {
        1
    }
}

impl<N: Unsigned + PowerOfTwo> Wall<N> {
    #[inline]
    fn x_offset() -> usize {
        N::USIZE.trailing_zeros() as usize + Self::y_offset()
    }

    fn to_index(&self) -> usize {
        ((self.x as usize) << Self::x_offset())
            | ((self.y as usize) << Self::y_offset())
            | self.top as usize
    }

    #[inline]
    fn max() -> u16 {
        N::U16 - 1
    }

    pub fn new(x: u16, y: u16, top: bool) -> Result<Self, OutOfBoundError> {
        if x <= Self::max() && y <= Self::max() {
            Ok(unsafe { Self::new_unchecked(x, y, top) })
        } else {
            Err(OutOfBoundError { x, y, top })
        }
    }

    pub unsafe fn new_unchecked(x: u16, y: u16, top: bool) -> Self {
        Self {
            x,
            y,
            top,
            _maze_width: PhantomData,
        }
    }
}

pub struct WallStorage<N>
where
    N: Mul<N>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<RefCell<Probability>>,
{
    walls: GenericArray<RefCell<Probability>, <<N as Mul<N>>::Output as Mul<U2>>::Output>,
    existence_threshold: Probability,
}

impl<N> WallStorage<N>
where
    N: Mul<N> + Unsigned + PowerOfTwo,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<RefCell<Probability>>,
{
    pub fn new(existence_threshold: Probability) -> Self {
        let walls = repeat_n(0.5, <<N as Mul<N>>::Output as Mul<U2>>::Output::USIZE)
            .map(|p| RefCell::new(unsafe { Probability::new_unchecked(p) }))
            .collect();
        let storage = Self {
            walls,
            existence_threshold,
        };
        storage.init();
        storage
    }

    pub fn init(&self) {
        let max = Wall::<N>::max();

        let update = |x: u16, y: u16, top: bool, prob: Probability| {
            *self.walls[Wall::<N>::new(x, y, top)
                .unwrap_or_else(|_| unreachable!())
                .to_index()]
            .borrow_mut() = prob;
        };

        for i in 0..max + 1 {
            //Walls on the top of maze exist.
            update(i, max, true, Probability::one());

            //Walls on the right of maze exist.
            update(max, i, false, Probability::one());
        }
        //A wall on the just right of start exists.
        update(0, 0, false, Probability::one());

        //A wall on the just top of start does not exist.
        update(0, 0, true, Probability::zero());
    }
}

pub enum WallStorageError {
    Borrow(core::cell::BorrowError),
    BorrowMut(core::cell::BorrowMutError),
}

impl From<core::cell::BorrowError> for WallStorageError {
    fn from(value: core::cell::BorrowError) -> Self {
        WallStorageError::Borrow(value)
    }
}

impl From<core::cell::BorrowMutError> for WallStorageError {
    fn from(value: core::cell::BorrowMutError) -> Self {
        WallStorageError::BorrowMut(value)
    }
}

//TODO: Write test.
impl<N> WallManager<Wall<N>> for WallStorage<N>
where
    N: Mul<N> + Unsigned + PowerOfTwo,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<RefCell<Probability>>,
{
    type Error = WallStorageError;

    fn try_existence_probability(&self, wall: &Wall<N>) -> Result<Probability, Self::Error> {
        Ok(self.walls[wall.to_index()].try_borrow()?.clone())
    }

    fn try_update(&self, wall: &Wall<N>, prob: &Probability) -> Result<(), Self::Error> {
        *self.walls[wall.to_index()].try_borrow_mut()? = *prob;
        Ok(())
    }

    #[inline]
    fn existence_threshold(&self) -> Probability {
        self.existence_threshold
    }
}

#[cfg(test)]
mod tests {
    use typenum::consts::*;

    use super::*;

    #[test]
    fn test_wall_to_index() {
        let test_cases = vec![
            (0, 0, false, 0),
            (0, 0, true, 1),
            (2, 3, true, 23),
            (3, 3, true, 31),
        ];

        for (x, y, top, expected) in test_cases {
            let wall = Wall::<U4>::new(x, y, top).unwrap();
            assert_eq!(wall.to_index(), expected);
        }
    }
}
