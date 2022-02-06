use core::fmt;

use heapless::{Deque, Vec};
use serde::{Deserialize, Serialize};

use crate::WIDTH;

/// A type for coordinate in maze.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Serialize, Deserialize)]
pub struct Coordinate<const W: u8> {
    pub x: u8,
    pub y: u8,
    pub is_top: bool,
}

impl<const W: u8> Coordinate<W> {
    pub fn new(x: u8, y: u8, is_top: bool) -> Option<Self> {
        if x >= W || y >= W {
            return None;
        }
        Some(Self { x, y, is_top })
    }

    pub fn as_index(&self) -> usize {
        ((self.y as usize) << (2 * W + 1)) | ((self.x as usize) << (W + 1)) | self.is_top as usize
    }

    // Return extended neighbors of the give coordinate.
    //
    // c: self, n: neighbor
    //
    // top
    // +---+-n-+
    // |   |   |
    //       .
    //       .
    // |   |   |
    // +---+-n-+
    // |   |   |
    // +---+-n-+
    // |   n   n
    // +---+-c-+
    // |   n   n
    // +---+-n-+
    // |   |   |
    // +---+-n-+
    // |   |   |
    //       .
    //       .
    // |   |   |
    // +---+-n-+
    //
    // right
    // +---   ---+---+-n-+-n-+---   ---+
    // n   ...   n   n   c   n   ...   n
    // +---   ---+---+-n-+-n-+---   ---+
    // |         |   |   |   |         |
    // +---   ---+---+---+---+---   ---+
    fn extended_neighbors(
        &self,
        filter: impl Fn(&Coordinate<W>) -> bool,
    ) -> Vec<Coordinate<W>, { WIDTH + 3 }> {
        let mut neighbors = Vec::new();
        let mut add_with_check = |x, y, is_top| {
            if let Some(coord) = self.new_relative(x, y, is_top) {
                if (filter)(&coord) {
                    neighbors.push(coord).unwrap();
                    return true;
                }
            }
            false
        };
        if self.is_top {
            add_with_check(0, 0, false);
            add_with_check(-1, 0, false);
            add_with_check(0, 1, false);
            add_with_check(-1, 1, false);
            for dy in 1.. {
                if !add_with_check(0, dy, true) {
                    break;
                }
            }
            for dy in 1.. {
                if !add_with_check(0, -dy, true) {
                    break;
                }
            }
        } else {
            add_with_check(0, 0, true);
            add_with_check(0, -1, true);
            add_with_check(1, 0, true);
            add_with_check(1, -1, true);
            for dx in 1.. {
                if !add_with_check(dx, 0, false) {
                    break;
                }
            }
            for dx in 1.. {
                if !add_with_check(-dx, 0, false) {
                    break;
                }
            }
        }
        neighbors
    }

    // Return all coordinates between `self` and `other`.
    //
    // If `self` is not a neighbor of `other`, this method panics.
    // The returned coordinates contain `self`, but do not contain `other`.
    fn intermediate_coords(&self, other: &Self) -> Vec<Coordinate<W>, { WIDTH - 2 }> {
        let mut coords = Vec::new();
        if self.is_top {
            if other.is_top {
                assert_eq!(self.x, other.x);
                assert_ne!(self.y, other.y);
                // `self.y` inclusive
                if self.y > other.y {
                    (other.y + 1)..(self.y + 1)
                } else {
                    self.y..other.y
                }
                .for_each(|y| {
                    coords
                        .push(Coordinate::new(self.x, y, true).unwrap())
                        .unwrap()
                });
            } else {
                let dx = other.x as i8 - self.x as i8;
                let dy = other.y as i8 - self.y as i8;
                match (dx, dy) {
                    (0, 0) | (-1, 0) | (-1, 1) | (0, 1) => coords.push(*self).unwrap(),
                    _ => unreachable!(),
                }
            }
        } else if other.is_top {
            let dx = other.x as i8 - self.x as i8;
            let dy = other.y as i8 - self.y as i8;
            match (dx, dy) {
                (0, 0) | (0, -1) | (1, 0) | (1, -1) => coords.push(*self).unwrap(),
                _ => unreachable!(),
            }
        } else {
            assert_ne!(self.x, other.x);
            assert_eq!(self.y, other.y);
            // `self.x` inclusive
            if self.x > other.x {
                (other.x + 1)..(self.x + 1)
            } else {
                self.x..other.x
            }
            .for_each(|x| {
                coords
                    .push(Coordinate::new(x, self.y, false).unwrap())
                    .unwrap()
            });
        }
        coords
    }

    // Return neighbors of the given coordinate.
    //
    // c: self, n: neighbor
    //
    // top
    // +---+-n-+
    // |   n   n
    // +---+-c-+
    // |   n   n
    // +---+-n-+
    // |   |   |
    // +---+---+
    //
    // right
    // +---+-n-+-n-+
    // |   n   c   n
    // +---+-n-+-n-+
    // |   |   |   |
    // +---+---+---+
    fn neighbors(&self, wall_state: impl Fn(&Coordinate<W>) -> WallState) -> Vec<Coordinate<W>, 6> {
        let mut neighbors = Vec::new();
        let mut add_with_check = |x, y, is_top| {
            if let Some(coord) = self.new_relative(x, y, is_top) {
                if !matches!(wall_state(&coord), WallState::Checked { exists: true }) {
                    neighbors.push(coord).unwrap();
                }
            }
        };
        if self.is_top {
            add_with_check(0, 0, false);
            add_with_check(0, -1, true);
            add_with_check(-1, 0, false);
            add_with_check(-1, 1, false);
            add_with_check(0, 1, false);
            add_with_check(0, 1, true);
        } else {
            add_with_check(0, 0, true);
            add_with_check(-1, 0, false);
            add_with_check(0, -1, true);
            add_with_check(1, -1, true);
            add_with_check(1, 0, false);
            add_with_check(1, 0, true);
        }
        neighbors
    }

    fn new_relative(&self, dx: i8, dy: i8, is_top: bool) -> Option<Self> {
        let &Self { x, y, .. } = self;
        let x = (x as i8).checked_add(dx)?;
        let y = (y as i8).checked_add(dy)?;
        if x.is_negative() || y.is_negative() {
            return None;
        }
        Some(Self {
            x: x as u8,
            y: y as u8,
            is_top,
        })
    }
}

pub struct Searcher<const W: u8> {
    start: Coordinate<W>,
    goal: Coordinate<W>,
}

/// State of wall.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Serialize, Deserialize)]
pub enum WallState {
    Unchecked,
    Checked { exists: bool },
}

/// Error on search.
#[derive(Debug)]
pub enum SearchError {
    Unreachable,
}

const QUE_MAX: usize = WIDTH * WIDTH * 2;

impl<T, const W: u8> core::ops::Index<Coordinate<W>> for [T] {
    type Output = T;

    fn index(&self, index: Coordinate<W>) -> &Self::Output {
        &(self)[index.as_index()]
    }
}

impl<T, const W: u8> core::ops::IndexMut<Coordinate<W>> for [T] {
    fn index_mut(&mut self, index: Coordinate<W>) -> &mut Self::Output {
        &mut (self)[index.as_index()]
    }
}

impl<const W: u8> Searcher<W> {
    pub fn new(start: Coordinate<W>, goal: Coordinate<W>) -> Self {
        Self { start, goal }
    }

    fn bfs_tree(
        &self,
        wall_state: impl Fn(&Coordinate<W>) -> WallState,
    ) -> [Option<Coordinate<W>>; QUE_MAX] {
        let mut que = Deque::<_, QUE_MAX>::new();
        let mut prev = [None; QUE_MAX];
        que.push_back(self.start).unwrap();
        prev[self.start] = Some(self.start);

        while let Some(node) = que.pop_front() {
            if node == self.goal {
                break;
            }
            for next in node.extended_neighbors(|coord| {
                !matches!(wall_state(coord), WallState::Checked { exists: true })
                    && prev[*coord].is_none()
            }) {
                prev[next] = Some(node);
                que.push_back(next).unwrap();
            }
        }
        prev
    }

    fn unchecked_walls(
        &self,
        wall_state: impl Fn(&Coordinate<W>) -> WallState,
        prev: &[Option<Coordinate<W>>],
    ) -> Vec<Coordinate<W>, QUE_MAX> {
        let mut walls = Vec::<_, QUE_MAX>::new();
        let mut cur = self.goal;
        while let Some(next) = prev[cur] {
            cur.intermediate_coords(&next)
                .into_iter()
                .filter(|node| wall_state(node) == WallState::Unchecked)
                .for_each(|node| walls.push(node).unwrap());
            if next == self.start {
                break;
            }
            cur = next;
        }
        walls
    }

    fn candidates(
        current: &Coordinate<W>,
        wall_state: impl Fn(&Coordinate<W>) -> WallState + Copy,
        unchecked_walls: &[Coordinate<W>],
    ) -> Vec<Coordinate<W>, 6> {
        let mut que = Deque::<_, QUE_MAX>::new();
        let mut is_visisted = [false; QUE_MAX];
        let mut candidates = Vec::<_, 6>::new();
        for &wall in unchecked_walls {
            que.push_back(wall).unwrap();
            is_visisted[wall] = true;
        }
        while let Some(node) = que.pop_front() {
            for next in node.neighbors(wall_state) {
                if next == *current {
                    candidates.push(node).unwrap();
                }
                if is_visisted[next] {
                    continue;
                }
                is_visisted[next] = true;
                que.push_back(next).unwrap();
            }
        }
        candidates
    }

    pub fn search(
        &self,
        current: &Coordinate<W>,
        wall_state: impl Fn(&Coordinate<W>) -> WallState + Copy,
    ) -> Result<Option<Commander<W>>, SearchError> {
        let prev = self.bfs_tree(wall_state);

        // Unreachable to goal from start.
        if prev[self.goal].is_none() {
            return Err(SearchError::Unreachable);
        }

        let walls = self.unchecked_walls(wall_state, &prev);

        // All walls between the path from start to goal are checked.
        if walls.is_empty() {
            return Ok(None);
        }

        let candidates = Self::candidates(current, wall_state, &walls);

        // Unreachable to all unchecked walls from current position.
        if candidates.is_empty() {
            return Err(SearchError::Unreachable);
        }

        Ok(Some(Commander { candidates }))
    }
}

pub struct Commander<const W: u8> {
    candidates: Vec<Coordinate<W>, 6>,
}

impl<const W: u8> Commander<W> {
    pub fn next_coordinate(
        &self,
        wall_state: impl Fn(&Coordinate<W>) -> WallState,
    ) -> Result<Option<Coordinate<W>>, SearchError> {
        for coord in &self.candidates {
            match wall_state(coord) {
                WallState::Checked { exists: false } => return Ok(Some(*coord)),
                WallState::Unchecked => return Ok(None),
                _ => (),
            }
        }
        Err(SearchError::Unreachable)
    }
}

const WALL_ARRAY_LEN: usize = WIDTH * WIDTH / 2;

#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Walls<const W: u8>([u8; WALL_ARRAY_LEN]);

impl<const W: u8> Default for Walls<W> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const W: u8> Walls<W> {
    pub fn new() -> Self {
        let mut walls = Self([0; WALL_ARRAY_LEN]);
        for i in 0..W {
            walls.update(
                &Coordinate {
                    x: W - 1,
                    y: i,
                    is_top: false,
                },
                &WallState::Checked { exists: true },
            );
            walls.update(
                &Coordinate {
                    x: i,
                    y: W - 1,
                    is_top: true,
                },
                &WallState::Checked { exists: true },
            );
        }
        walls
    }

    pub fn update(&mut self, coord: &Coordinate<W>, state: &WallState) {
        let bit = match state {
            WallState::Unchecked => 0,
            WallState::Checked { exists: false } => 1,
            WallState::Checked { exists: true } => 3,
        };
        let index = coord.as_index();
        let shift = (index % 4) * 2;
        (self.0)[index / 4] &= !(3 << shift);
        (self.0)[index / 4] |= bit << shift;
    }

    pub fn wall_state(&self, wall: &Coordinate<W>) -> WallState {
        let index = wall.as_index();
        let bit = ((self.0)[index / 4] >> ((index % 4) * 2)) & 3;
        match bit {
            0 => WallState::Unchecked,
            1 => WallState::Checked { exists: false },
            3 => WallState::Checked { exists: true },
            _ => unreachable!(),
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub struct ParseWallsError {
    kind: ParseWallsErrorKind,
    line: usize,
    char_range: (usize, usize),
}

#[derive(Debug, PartialEq, Eq)]
enum ParseWallsErrorKind {
    One { expected: &'static str },
    Two { expected: [&'static str; 2] },
}

impl ParseWallsError {
    fn new(expected: &'static str, line: usize, char_range: (usize, usize)) -> Self {
        Self {
            kind: ParseWallsErrorKind::One { expected },
            line,
            char_range,
        }
    }

    fn new_two(expected: [&'static str; 2], line: usize, char_range: (usize, usize)) -> Self {
        Self {
            kind: ParseWallsErrorKind::Two { expected },
            line,
            char_range,
        }
    }
}

impl fmt::Display for ParseWallsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "line {}, character {} to {}, ",
            self.line + 1,
            self.char_range.0,
            self.char_range.1
        )?;
        match self.kind {
            ParseWallsErrorKind::One { expected } => write!(f, "here must be `{}`.", expected)?,
            ParseWallsErrorKind::Two { expected } => {
                write!(f, "here must be `{}` or `{}`.", expected[0], expected[1])?
            }
        }
        Ok(())
    }
}

impl<const W: u8> core::str::FromStr for Walls<W> {
    type Err = ParseWallsError;

    // TODO: validation
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut walls = Self::new();
        let mut update = |x, y, is_top, exists| {
            walls.update(&Coordinate { x, y, is_top }, &WallState::Checked { exists });
        };
        let check = |s: &str, expected, len, y, x| {
            if s.get(..len) == Some(expected) {
                Ok(())
            } else {
                Err(ParseWallsError::new(expected, y, (x, x + len)))
            }
        };

        for (y, mut s) in s.lines().enumerate().take(2 * W as usize + 1) {
            let y = y as u8;
            if y == 2 * W {
                check(s, "+", 1, y as usize, 0)?;
                s = &s[1..];
                for x in 0..W {
                    check(s, "---+", 4, y as usize, 4 * x as usize + 1)?;
                    s = &s[4..];
                }
                break;
            }
            for x in 0..W {
                if y & 1 == 0 {
                    if x == 0 {
                        check(s, "+", 1, y as usize, x as usize)?;
                        s = &s[1..];
                    }
                    match s.get(..4) {
                        Some("---+") => update(x, W - y / 2 - 1, true, true),
                        Some("   +") => update(x, W - y / 2 - 1, true, false),
                        _ => {
                            return Err(ParseWallsError::new_two(
                                ["---+", "   +"],
                                y as usize,
                                (4 * x as usize + 1, 4 * x as usize + 5),
                            ))
                        }
                    }
                    s = &s[4..];
                } else {
                    if x == 0 {
                        check(s, "|", 1, y as usize, x as usize)?;
                        s = &s[1..];
                    }
                    match s.get(..4) {
                        Some("   |") => update(x, W - y / 2 - 1, false, true),
                        Some("    ") => update(x, W - y / 2 - 1, false, false),
                        _ => {
                            return Err(ParseWallsError::new_two(
                                ["   |", "    "],
                                y as usize,
                                (4 * x as usize + 1, 4 * x as usize + 5),
                            ))
                        }
                    }
                    s = &s[4..];
                }
            }
        }
        Ok(walls)
    }
}

impl<const W: u8> fmt::Display for Walls<W> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let exists = |x, y, is_top| {
            matches!(
                self.wall_state(&Coordinate::new(x, y, is_top).unwrap()),
                WallState::Checked { exists: true }
            )
        };
        for y in (0..W).rev() {
            for x in 0..W {
                if exists(x, y, true) {
                    write!(f, "+---")?;
                } else {
                    write!(f, "+   ")?;
                }
            }
            writeln!(f, "+")?;

            write!(f, "|")?;
            for x in 0..W {
                if exists(x, y, false) {
                    write!(f, "   |")?;
                } else {
                    write!(f, "    ")?;
                }
            }
            writeln!(f)?;
        }
        for _ in 0..W {
            write!(f, "+---")?;
        }
        writeln!(f, "+")?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    #[test]
    fn test_empty_display() {
        let s = Walls::<4>::new().to_string();
        let expected = include_str!("../mazes/empty4.dat");
        assert_eq!(&s, expected);
    }

    #[test]
    fn test_simple_display() {
        let mut walls = Walls::<4>::new();
        let vec = vec![
            (0, 0, false),
            (0, 1, false),
            (0, 2, false),
            (1, 2, true),
            (2, 2, true),
            (2, 2, false),
            (1, 1, false),
            (2, 0, false),
            (2, 0, true),
        ];
        for (x, y, is_top) in vec {
            walls.update(
                &Coordinate { x, y, is_top },
                &WallState::Checked { exists: true },
            );
        }
        let s = walls.to_string();
        let expected = include_str!("../mazes/maze4_1.dat");
        assert_eq!(&s, expected);
    }

    #[test]
    fn test_parse() {
        let test_cases = vec![
            include_str!("../mazes/maze4_1.dat"),
            include_str!("../mazes/maze4_2.dat"),
            include_str!("../mazes/maze4_3.dat"),
        ];

        for s in test_cases {
            let s2 = s.parse::<Walls<4>>().unwrap().to_string();
            assert_eq!(&s2, s);
        }
    }

    #[test]
    fn test_parse_error() {
        let test_cases = vec![
            (
                include_str!("../mazes/invalid4_1.dat"),
                Err(ParseWallsError::new_two(["---+", "   +"], 0, (5, 9))),
            ),
            (
                include_str!("../mazes/invalid4_2.dat"),
                Err(ParseWallsError::new_two(["   |", "    "], 7, (5, 9))),
            ),
        ];

        for (input, expected) in test_cases {
            assert_eq!(input.parse::<Walls<4>>(), expected);
        }
    }

    fn restore_path<const W: u8>(
        start: &Coordinate<W>,
        goal: &Coordinate<W>,
        prev: &[Option<Coordinate<W>>],
    ) -> Vec<Coordinate<W>> {
        let mut path = Vec::new();
        let mut cur = *goal;
        while let Some(next) = prev[cur] {
            path.push(cur);
            if &cur == start {
                break;
            }
            cur = next;
        }
        path.reverse();
        path
    }

    fn new_coord<const W: u8>((x, y, is_top): (u8, u8, bool)) -> Coordinate<W> {
        Coordinate::new(x, y, is_top).unwrap()
    }

    #[test]
    fn test_shortest_path() {
        let test_cases = vec![(
            (0, 0, true),
            (1, 0, true),
            include_str!("../mazes/maze4_1.dat"),
            vec![
                (0, 0, true),
                (0, 2, true),
                (0, 3, false),
                (2, 3, false),
                (3, 2, true),
                (3, 1, true),
                (2, 1, false),
                (2, 1, true),
                (1, 2, false),
                (1, 1, true),
                (1, 0, true),
            ],
        )];

        const W: u8 = 4;
        for (start, goal, walls, expected) in test_cases {
            let start = new_coord(start);
            let goal = new_coord(goal);
            let walls = walls.parse::<Walls<W>>().unwrap();
            let expected = expected.into_iter().map(new_coord::<W>).collect::<Vec<_>>();
            let searcher = Searcher::<W>::new(start, goal);
            let prev = searcher.bfs_tree(|coord| walls.wall_state(coord));
            let path = restore_path(&start, &goal, &prev);
            assert_eq!(path.as_slice(), expected.as_slice());
        }
    }

    #[test]
    fn test_unchecked_wall() {
        let test_cases = vec![(
            (0, 0, true),
            (1, 0, true),
            include_str!("../mazes/maze4_1.dat"),
            vec![
                (0, 1, true),
                (1, 0, false),
                (0, 2, true),
                (1, 3, false),
                (2, 3, false),
                (2, 1, false),
            ],
            vec![
                (0, 1, true),
                (0, 2, true),
                (1, 3, false),
                (2, 3, false),
                (2, 1, false),
            ],
        )];

        const W: u8 = 4;
        for (start, goal, walls, unchecked_walls, expected) in test_cases {
            let start = new_coord(start);
            let goal = new_coord(goal);
            let mut walls = walls.parse::<Walls<W>>().unwrap();
            for wall in unchecked_walls {
                walls.update(&new_coord(wall), &WallState::Unchecked);
            }
            let mut expected = expected.into_iter().map(new_coord::<W>).collect::<Vec<_>>();
            expected.sort();
            let searcher = Searcher::<W>::new(start, goal);
            let prev = searcher.bfs_tree(|coord| walls.wall_state(coord));
            let mut walls = searcher.unchecked_walls(|coord| walls.wall_state(coord), &prev);
            walls.sort();
            assert_eq!(walls.as_slice(), expected.as_slice());
        }
    }

    #[test]
    fn test_search() {
        let test_cases = vec![(
            (0, 0, true),
            (1, 0, true),
            include_str!("../mazes/maze4_1.dat"),
            vec![
                (0, 1, true),
                (1, 0, false),
                (0, 2, true),
                (1, 3, false),
                (2, 3, false),
                (2, 1, true),
            ],
            (3, 0, true),
            vec![(2, 1, false), (3, 1, true)],
        )];

        const W: u8 = 4;
        for (start, goal, walls, unchecked_walls, current, expected) in test_cases {
            let start = new_coord(start);
            let goal = new_coord(goal);
            let mut walls = walls.parse::<Walls<W>>().unwrap();
            for wall in unchecked_walls {
                walls.update(&new_coord(wall), &WallState::Unchecked);
            }
            let current = new_coord(current);
            let expected = expected.into_iter().map(new_coord::<W>).collect::<Vec<_>>();
            let searcher = Searcher::<W>::new(start, goal);
            let Commander { candidates } = searcher
                .search(&current, |coord| walls.wall_state(coord))
                .unwrap()
                .unwrap();
            assert_eq!(candidates.as_slice(), expected.as_slice());
        }
    }
}
