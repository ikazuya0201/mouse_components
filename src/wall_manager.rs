use core::cell::RefCell;
use core::marker::PhantomData;
use core::ops::Mul;

use generic_array::{ArrayLength, GenericArray};
use typenum::{consts::U2, PowerOfTwo, Unsigned};

use crate::simple_maze::WallManager as IWallManager;
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

    pub unsafe fn new_unchecked(x: u16, y: u16, top: bool) -> Self {
        Self {
            x,
            y,
            top,
            _maze_width: PhantomData,
        }
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
}

#[derive(Clone)]
pub struct WallManager<N>
where
    N: Mul<N>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<RefCell<Probability>>,
{
    walls: GenericArray<RefCell<Probability>, <<N as Mul<N>>::Output as Mul<U2>>::Output>,
    existence_threshold: Probability,
}

impl<N> core::fmt::Debug for WallManager<N>
where
    N: Mul<N> + Unsigned + PowerOfTwo,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<RefCell<Probability>>,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let prob = |x: u16, y: u16, is_top: bool| -> Probability {
            let wall = Wall::<N>::new(x, y, is_top).unwrap();
            self.walls[wall.to_index()].borrow().clone()
        };
        writeln!(f, "")?;
        for y in (0..N::U16).rev() {
            write!(f, " +--")?;
            for x in 0..N::U16 - 1 {
                write!(f, "{:1.1}--+--", f32::from(prob(x, y, true)))?;
            }
            writeln!(f, "{:1.1}--+", f32::from(prob(N::U16 - 1, y, true)))?;

            write!(f, " |  ")?;
            for _ in 0..N::U16 {
                write!(f, "     |  ")?;
            }
            writeln!(f, "")?;

            write!(f, "1.0 ")?;
            for x in 0..N::U16 {
                write!(f, "    {:1.1} ", f32::from(prob(x, y, false)))?;
            }
            writeln!(f, "")?;

            write!(f, " |  ")?;
            for _ in 0..N::U16 {
                write!(f, "     |  ")?;
            }
            writeln!(f, "")?;
        }
        write!(f, " +--")?;
        for _ in 0..N::U16 - 1 {
            write!(f, "1.0--+--")?;
        }
        writeln!(f, "1.0--+")?;
        Ok(())
    }
}

impl<N> WallManager<N>
where
    N: Mul<N> + Unsigned + PowerOfTwo,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<RefCell<Probability>>,
{
    fn _new(existence_threshold: Probability) -> Self {
        let walls = repeat_n(0.5, <<N as Mul<N>>::Output as Mul<U2>>::Output::USIZE)
            .map(|p| RefCell::new(unsafe { Probability::new_unchecked(p) }))
            .collect();
        Self {
            walls,
            existence_threshold,
        }
    }

    ///Initialize walls like following.
    ///- All walls surrounding the maze exist with probability 1.
    ///- A wall just rightside of the start position exists with probability 1
    ///(the start position is left-bottom of the maze).
    ///- A wall just forward of the start position does not exist (probability 0).
    ///- Otherwise exist with probability 0.5.
    pub fn new(existence_threshold: Probability) -> Self {
        let storage = Self::_new(existence_threshold);
        storage.init();
        storage
    }

    fn init(&self) {
        let max = Wall::<N>::max();

        for i in 0..max + 1 {
            //Walls on the top of maze exist.
            self.update(i, max, true, Probability::one());

            //Walls on the right of maze exist.
            self.update(max, i, false, Probability::one());
        }
        //A wall on the just right of start exists.
        self.update(0, 0, false, Probability::one());

        //A wall on the just top of start does not exist.
        self.update(0, 0, true, Probability::zero());
    }

    fn update(&self, x: u16, y: u16, top: bool, prob: Probability) {
        *self.walls[Wall::<N>::new(x, y, top)
            .unwrap_or_else(|err| unreachable!("This is bug: {:?}", err))
            .to_index()]
        .borrow_mut() = prob;
    }

    ///NOTE: Input str of this method must be formatted like following.
    ///- Input str should not have unnecessary characters.
    ///- All walls are initialized with existence probability 0 or 1.
    ///- Pillars must be notated as `+`.
    ///- Horizontal walls must be notated as `---` (3 hyphens).
    ///- Vertical walls must be notated as `|`.
    ///- If a wall does not exist in that position,
    ///replace the corresponding wall characters with the same number of spaces.
    ///(`---` is replaced with `   `, `|` is replaced with ` `)
    ///- All squares must be notated as `   ` (3 spaces).
    ///- Regulations of micro mouse contests can be ignored.
    ///
    ///examples:
    ///```example1
    ///+---+---+
    ///|   |   |
    ///+---+   +
    ///|       |
    ///+---+---+
    ///```
    ///```example2
    ///+---+---+---+---+
    ///|               |
    ///+   +---+---+   +
    ///|   |       |   |
    ///+   +   +   +   +
    ///|   |   |       |
    ///+   +   +---+   +
    ///|   |       |   |
    ///+---+---+---+---+
    ///```
    pub fn with_str(existence_threshold: Probability, input: &str) -> Self {
        let storage = Self::_new(existence_threshold);
        storage.init_with_str(input);
        storage
    }

    fn init_with_str(&self, input: &str) {
        input
            .lines()
            .enumerate()
            .take(2 * N::USIZE) //not consider bottom line
            .for_each(|(y, line)| {
                let y = y as u16;
                line.chars().enumerate().skip(1).for_each(|(x, c)| {
                    //not consider left line
                    let x = x as u16;
                    if y % 2 == 0 {
                        //check top walls
                        if x % 4 != 1 {
                            return;
                        }
                        let x = x / 4;
                        if c == '-' {
                            self.update(x, N::U16 - y / 2 - 1, true, Probability::one());
                        } else {
                            self.update(x, N::U16 - y / 2 - 1, true, Probability::zero());
                        }
                    } else {
                        //check right walls
                        if x % 4 != 0 {
                            return;
                        }
                        let x = x / 4;
                        if c == '|' {
                            self.update(x - 1, N::U16 - y / 2 - 1, false, Probability::one());
                        } else {
                            self.update(x - 1, N::U16 - y / 2 - 1, false, Probability::zero());
                        }
                    }
                });
            });
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
impl<N> IWallManager<Wall<N>> for WallManager<N>
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

    #[test]
    fn test_with_str() {
        let input = "+---+---+
|   |   |
+---+   +
|       |
+---+---+";

        type Size = U2;

        let wall_manager = WallManager::<Size>::with_str(Probability::new(0.1).unwrap(), input);

        let walls = vec![
            ((0, 0, false), false),
            ((0, 0, true), true),
            ((0, 1, false), true),
            ((0, 1, true), true),
            ((1, 0, false), true),
            ((1, 0, true), false),
            ((1, 1, false), true),
            ((1, 1, true), true),
        ];

        for (wall, expected) in walls {
            let wall = Wall::<Size>::new(wall.0, wall.1, wall.2).unwrap();
            assert_eq!(wall_manager.exists(&wall), expected, "{:?}", wall);
        }
    }
}
