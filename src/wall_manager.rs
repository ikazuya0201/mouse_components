//! An implementation of [WallChecker](crate::mazes::WallChecker) and
//! [WallProbabilityManager](crate::wall_detector::WallProbabilityManager).

use core::fmt;

use heapless::Vec;
use serde::{Deserialize, Serialize};
use spin::Mutex;

use crate::mazes::WallChecker;
use crate::nodes::SearchNode;
use crate::utils::probability::Probability;
use crate::wall_detector::WallProbabilityManager;
use crate::MAZE_WIDTH_UPPER_BOUND;

pub(crate) const WALL_NUMBER_UPPER_BOUND: usize =
    2 * MAZE_WIDTH_UPPER_BOUND * MAZE_WIDTH_UPPER_BOUND;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct Wall<const N: usize> {
    x: u8,
    y: u8,
    top: bool,
}

impl<const N: usize> From<Wall<N>> for [SearchNode<N>; 2] {
    fn from(value: Wall<N>) -> [SearchNode<N>; 2] {
        use crate::types::data::AbsoluteDirection::*;

        let (dx, dy, dirs) = if value.is_top() {
            (0, 1, [North, South])
        } else {
            (1, 0, [East, West])
        };
        let x = (value.x() * 2 + dx) as i8;
        let y = (value.y() * 2 + dy) as i8;
        let new = |x, y, dir| {
            SearchNode::<N>::new(x, y, dir)
                .unwrap_or_else(|err| unreachable!("Should never be error: {:?}", err))
        };
        [new(x, y, dirs[0]), new(x, y, dirs[1])]
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct OutOfBoundError {
    x: u8,
    y: u8,
    top: bool,
}

impl<const N: usize> Wall<N> {
    pub fn x(&self) -> u8 {
        self.x
    }

    pub fn y(&self) -> u8 {
        self.y
    }

    pub fn is_top(&self) -> bool {
        self.top
    }

    const fn y_offset() -> usize {
        1
    }

    /// Creates a new `Wall` with no check of boundaries.
    ///
    /// # Safety
    /// `x` and `y` should be smaller than `N`.
    pub unsafe fn new_unchecked(x: u8, y: u8, top: bool) -> Self {
        Self { x, y, top }
    }
}

impl<const N: usize> Wall<N> {
    const fn x_offset() -> usize {
        N.trailing_zeros() as usize + Self::y_offset()
    }

    fn as_index(&self) -> usize {
        ((self.x as usize) << Self::x_offset())
            | ((self.y as usize) << Self::y_offset())
            | self.top as usize
    }

    const fn max() -> u8 {
        N as u8 - 1
    }

    pub fn new(x: u8, y: u8, top: bool) -> Result<Self, OutOfBoundError> {
        if x <= Self::max() && y <= Self::max() {
            Ok(unsafe { Self::new_unchecked(x, y, top) })
        } else {
            Err(OutOfBoundError { x, y, top })
        }
    }
}

pub struct WallManager<const N: usize> {
    walls: Vec<Mutex<Probability>, WALL_NUMBER_UPPER_BOUND>,
    existence_threshold: Probability,
}

impl<const N: usize> fmt::Debug for WallManager<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("WallManager")
            .field("walls", &self.walls)
            .field("existence_threshold", &self.existence_threshold)
            .finish()
    }
}

impl<const N: usize> fmt::Display for WallManager<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let prob = |x: u8, y: u8, is_top: bool| -> Probability {
            let wall = Wall::<N>::new(x, y, is_top).unwrap();
            *self.walls[wall.as_index()].lock()
        };
        writeln!(f)?;
        for y in (0..N as u8).rev() {
            write!(f, " +--")?;
            for x in 0..N as u8 - 1 {
                write!(f, "{:1.1}--+--", f32::from(prob(x, y, true)))?;
            }
            writeln!(f, "{:1.1}--+", f32::from(prob(N as u8 - 1, y, true)))?;

            write!(f, " |  ")?;
            for _ in 0..N as u8 {
                write!(f, "     |  ")?;
            }
            writeln!(f)?;

            write!(f, "1.0 ")?;
            for x in 0..N as u8 {
                write!(f, "    {:1.1} ", f32::from(prob(x, y, false)))?;
            }
            writeln!(f)?;

            write!(f, " |  ")?;
            for _ in 0..N as u8 {
                write!(f, "     |  ")?;
            }
            writeln!(f)?;
        }
        write!(f, " +--")?;
        for _ in 0..N as u8 - 1 {
            write!(f, "1.0--+--")?;
        }
        writeln!(f, "1.0--+")?;
        Ok(())
    }
}

impl<const N: usize> WallManager<N> {
    fn _new(existence_threshold: Probability) -> Self {
        let walls = core::iter::repeat_with(|| {
            Mutex::new(Probability::new(0.5).expect("Should never panic"))
        })
        .take(WALL_NUMBER_UPPER_BOUND)
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
            self._update(i, max, true, Probability::one());

            //Walls on the right of maze exist.
            self._update(max, i, false, Probability::one());
        }
        //A wall on the just right of start exists.
        self._update(0, 0, false, Probability::one());

        //A wall on the just top of start does not exist.
        self._update(0, 0, true, Probability::zero());
    }

    fn _update(&self, x: u8, y: u8, top: bool, prob: Probability) {
        *self.walls[Wall::<N>::new(x, y, top)
            .unwrap_or_else(|err| unreachable!("This is bug: {:?}", err))
            .as_index()]
        .lock() = prob;
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
            .take(2 * N) //not consider bottom line
            .for_each(|(y, line)| {
                let y = y as u8;
                line.chars().enumerate().skip(1).for_each(|(x, c)| {
                    //not consider left line
                    let x = x as u8;
                    if y % 2 == 0 {
                        //check top walls
                        if x % 4 != 1 {
                            return;
                        }
                        let x = x / 4;
                        if c == '-' {
                            self._update(x, N as u8 - y / 2 - 1, true, Probability::one());
                        } else {
                            self._update(x, N as u8 - y / 2 - 1, true, Probability::zero());
                        }
                    } else {
                        //check right walls
                        if x % 4 != 0 {
                            return;
                        }
                        let x = x / 4;
                        if c == '|' {
                            self._update(x - 1, N as u8 - y / 2 - 1, false, Probability::one());
                        } else {
                            self._update(x - 1, N as u8 - y / 2 - 1, false, Probability::zero());
                        }
                    }
                });
            });
    }

    pub fn update(&self, wall: &Wall<N>, prob: &Probability) {
        while self.try_update(wall, prob).is_err() {}
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MutexError;

//TODO: Write test.
impl<const N: usize> WallProbabilityManager<Wall<N>> for WallManager<N> {
    type Error = MutexError;

    fn try_existence_probability(&self, wall: &Wall<N>) -> Result<Probability, Self::Error> {
        Ok(*self.walls[wall.as_index()].try_lock().ok_or(MutexError)?)
    }

    fn try_update(&self, wall: &Wall<N>, prob: &Probability) -> Result<(), Self::Error> {
        *self.walls[wall.as_index()].try_lock().ok_or(MutexError)? = *prob;
        Ok(())
    }
}

impl<const N: usize> WallChecker<Wall<N>> for WallManager<N> {
    type Error = MutexError;

    fn try_is_checked(&self, wall: &Wall<N>) -> Result<bool, Self::Error> {
        let prob = self.try_existence_probability(wall)?;
        Ok(prob < self.existence_threshold || prob > self.existence_threshold.reverse())
    }

    fn try_exists(&self, wall: &Wall<N>) -> Result<bool, Self::Error> {
        let prob = self.try_existence_probability(wall)?;
        Ok(prob > self.existence_threshold.reverse())
    }
}

#[cfg(test)]
mod tests {
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
            let wall = Wall::<4>::new(x, y, top).unwrap();
            assert_eq!(wall.as_index(), expected);
        }
    }

    #[test]
    fn test_with_str() {
        let input = "+---+---+
|   |   |
+---+   +
|       |
+---+---+";

        const SIZE: usize = 2;

        let wall_manager = WallManager::<SIZE>::with_str(Probability::new(0.1).unwrap(), input);

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
            let wall = Wall::<SIZE>::new(wall.0, wall.1, wall.2).unwrap();
            assert_eq!(wall_manager.exists(&wall), expected, "{:?}", wall);
        }
    }
}
