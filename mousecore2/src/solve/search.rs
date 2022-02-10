use core::ops::ControlFlow;

use heapless::{Deque, Vec};
use serde::{Deserialize, Serialize};

use crate::WIDTH;

const FIL_LEN: usize = WIDTH * WIDTH / 4;

struct Filter([u8; FIL_LEN]);

impl Filter {
    fn new() -> Self {
        Self([0; FIL_LEN])
    }

    fn with_coord<const W: u8>(coords: &[Coordinate<W>]) -> Self {
        let mut filter = Filter::new();
        for coord in coords {
            filter.set(coord);
        }
        filter
    }

    fn set<const W: u8>(&mut self, coord: &Coordinate<W>) {
        let index = coord.as_index();
        (self.0)[index >> 3] |= 1 << (index & 7);
    }

    fn contains<const W: u8>(&self, coord: &Coordinate<W>) -> bool {
        let index = coord.as_index();
        ((self.0)[index >> 3] >> (index & 7)) & 1 == 1
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum NeighborKind {
    VerticalOrHorizontal,
    BottomLeftToTopRight,
    TopLeftToBottomRight,
}

/// A type for coordinate in maze.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Serialize, Deserialize)]
pub struct Coordinate<const W: u8> {
    x: u8,
    y: u8,
    is_top: bool,
}

impl<const W: u8> Coordinate<W> {
    pub fn new(x: u8, y: u8, is_top: bool) -> Option<Self> {
        if x >= W || y >= W || !W.is_power_of_two() {
            return None;
        }
        Some(Self { x, y, is_top })
    }

    pub fn as_index(&self) -> usize {
        ((self.y as usize) << (W.trailing_zeros() + 1))
            | ((self.x as usize) << 1)
            | self.is_top as usize
    }

    pub(crate) fn x(&self) -> u8 {
        self.x
    }

    pub(crate) fn y(&self) -> u8 {
        self.y
    }

    pub(crate) fn is_top(&self) -> bool {
        self.is_top
    }

    // Return extended neighbors of the give coordinate.
    //
    // c: self, n: neighbor
    // same for diagonal
    //
    // top
    // +---+-n-+
    // |   |   |
    //       .
    //       .
    // |   |   |
    // +---+-n-+
    // n   |   |
    // +-n-+-n-+
    // |   n   n
    // +---+-c-+
    // |   n   n
    // +-n-+-n-+
    // n   |   |
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
    // |         |   n   |   n         |
    // +---   ---+-n-+---+---+-n-   ---+
    fn extended_neighbors(
        &self,
        filter: impl Fn(&Coordinate<W>) -> bool,
        exclude: Option<NeighborKind>,
    ) -> Vec<(Coordinate<W>, NeighborKind), { 6 * WIDTH }> {
        use NeighborKind::*;

        let mut neighbors = Vec::new();
        let mut add_with_check = |x, y, is_top, kind| {
            if Some(kind) == exclude {
                return ControlFlow::Break(());
            }
            if let Some(coord) = self.new_relative(x, y, is_top) {
                if (filter)(&coord) {
                    neighbors.push((coord, kind)).unwrap();
                    return ControlFlow::Continue(());
                }
            }
            ControlFlow::Break(())
        };
        macro_rules! check_seq {
            ($f: expr) => {
                (1..).try_for_each($f);
            };
        }
        if self.is_top {
            // top right
            check_seq!(|i| {
                add_with_check(i - 1, i, false, BottomLeftToTopRight)?;
                add_with_check(i, i, true, BottomLeftToTopRight)
            });
            // bottom left
            check_seq!(|i: i8| {
                add_with_check(-i, 1 - i, false, BottomLeftToTopRight)?;
                add_with_check(-i, -i, true, BottomLeftToTopRight)
            });
            // top left
            check_seq!(|i: i8| {
                add_with_check(-i, i, false, TopLeftToBottomRight)?;
                add_with_check(-i, i, true, TopLeftToBottomRight)
            });
            // bottom right
            check_seq!(|i: i8| {
                add_with_check(i - 1, 1 - i, false, TopLeftToBottomRight)?;
                add_with_check(i, -i, true, TopLeftToBottomRight)
            });
            // top
            check_seq!(|i| add_with_check(0, i, true, VerticalOrHorizontal));
            // bottom
            check_seq!(|i: i8| add_with_check(0, -i, true, VerticalOrHorizontal));
        } else {
            // top right
            check_seq!(|i| {
                add_with_check(i, i - 1, true, BottomLeftToTopRight)?;
                add_with_check(i, i, false, BottomLeftToTopRight)
            });
            // bottom left
            check_seq!(|i: i8| {
                add_with_check(1 - i, -i, true, BottomLeftToTopRight)?;
                add_with_check(-i, -i, false, BottomLeftToTopRight)
            });
            // top left
            check_seq!(|i: i8| {
                add_with_check(1 - i, i - 1, true, TopLeftToBottomRight)?;
                add_with_check(-i, i, false, TopLeftToBottomRight)
            });
            // bottom right
            check_seq!(|i: i8| {
                add_with_check(i, -i, true, TopLeftToBottomRight)?;
                add_with_check(i, -i, false, TopLeftToBottomRight)
            });
            // right
            check_seq!(|i| add_with_check(i, 0, false, VerticalOrHorizontal));
            // left
            check_seq!(|i: i8| add_with_check(-i, 0, false, VerticalOrHorizontal));
        }
        neighbors
    }

    // Return all coordinates between `self` and `other`.
    //
    // If `self` is not a neighbor of `other`, this method panics.
    // The returned coordinates contain `self`, but do not contain `other`.
    fn intermediate_coords(&self, other: &Self) -> Vec<Coordinate<W>, { 2 * WIDTH }> {
        let mut coords = Vec::new();
        coords.push(*self).unwrap();

        let mut add_with_check = |x, y, is_top| {
            if let Some(coord) = self.new_relative(x, y, is_top) {
                if &coord != other {
                    coords.push(coord).unwrap();
                    return ControlFlow::Continue(());
                }
            }
            ControlFlow::Break(())
        };
        macro_rules! add_seq {
            ($f: expr) => {
                (1..).try_for_each($f);
            };
        }

        let dx = other.x as i8 - self.x as i8;
        let dy = other.y as i8 - self.y as i8;
        if self.is_top {
            if dx >= 0 && ((!other.is_top && dx + 1 == dy) || (other.is_top && dx == dy)) {
                // top right
                add_seq!(|i| {
                    add_with_check(i - 1, i, false)?;
                    add_with_check(i, i, true)
                });
            } else if dx < 0 && ((!other.is_top && dx + 1 == dy) || (other.is_top && dx == dy)) {
                // bottom left
                add_seq!(|i: i8| {
                    add_with_check(-i, 1 - i, false)?;
                    add_with_check(-i, -i, true)
                });
            } else if dx < 0 && dx == -dy {
                // top left
                add_seq!(|i: i8| {
                    add_with_check(-i, i, false)?;
                    add_with_check(-i, i, true)
                });
            } else if dx >= 0 && dx == -dy {
                // bottom right
                add_seq!(|i: i8| {
                    add_with_check(i - 1, 1 - i, false)?;
                    add_with_check(i, -i, true)
                });
            } else if dx == 0 && dy > 0 && other.is_top {
                // top
                add_seq!(|i| add_with_check(0, i, true));
            } else if dx == 0 && dy < 0 && other.is_top {
                // bottom
                add_seq!(|i: i8| add_with_check(0, -i, true));
            } else {
                unreachable!()
            }
        } else if dy >= 0 && ((dx == dy + 1 && other.is_top) || (dx == dy && !other.is_top)) {
            // top right
            add_seq!(|i| {
                add_with_check(i, i - 1, true)?;
                add_with_check(i, i, false)
            });
        } else if dy < 0 && ((dx == dy + 1 && other.is_top) || (dx == dy && !other.is_top)) {
            // bottom left
            add_seq!(|i: i8| {
                add_with_check(1 - i, -i, true)?;
                add_with_check(-i, -i, false)
            });
        } else if dy >= 0 && dx == -dy {
            // top left
            add_seq!(|i: i8| {
                add_with_check(1 - i, i - 1, true)?;
                add_with_check(-i, i, false)
            });
        } else if dy < 0 && dx == -dy {
            // bottom right
            add_seq!(|i: i8| {
                add_with_check(i, -i, true)?;
                add_with_check(i, -i, false)
            });
        } else if dx > 0 && dy == 0 && !other.is_top {
            // right
            add_seq!(|i| add_with_check(i, 0, false));
        } else if dx < 0 && dy == 0 && !other.is_top {
            // left
            add_seq!(|i: i8| add_with_check(-i, 0, false));
        } else {
            unreachable!()
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

    fn is_neighbor(&self, other: &Self) -> bool {
        let dx = other.x as i8 - self.x as i8;
        let dy = other.y as i8 - self.y as i8;
        matches!(
            (dx, dy, self.is_top, other.is_top),
            (0, 0, true, false)
                | (0, -1, true, true)
                | (-1, 0, true, false)
                | (-1, 1, true, false)
                | (0, 1, true, false)
                | (0, 1, true, true)
                | (0, 0, false, true)
                | (-1, 0, false, false)
                | (0, -1, false, true)
                | (1, -1, false, true)
                | (1, 0, false, false)
                | (1, 0, false, true)
        )
    }

    fn new_relative(&self, dx: i8, dy: i8, is_top: bool) -> Option<Self> {
        let &Self { x, y, .. } = self;
        let x = (x as i8).checked_add(dx)?;
        let y = (y as i8).checked_add(dy)?;
        if x.is_negative() || y.is_negative() || x >= W as i8 || y >= W as i8 {
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
    goal_filter: Filter,
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
    pub fn new(start: Coordinate<W>, goals: &[Coordinate<W>]) -> Self {
        Self {
            start,
            goal_filter: Filter::with_coord(goals),
        }
    }

    fn bfs_tree(
        &self,
        is_wall: impl Fn(&Coordinate<W>) -> bool,
    ) -> Option<([Option<Coordinate<W>>; QUE_MAX], Coordinate<W>)> {
        let mut que = Deque::<_, QUE_MAX>::new();
        let mut prev = [None; QUE_MAX];
        que.push_back((self.start, None)).unwrap();
        prev[self.start] = Some(self.start);

        while let Some((node, kind)) = que.pop_front() {
            for (next, kind) in node.extended_neighbors(|coord| !is_wall(coord), kind) {
                if prev[next].is_some() {
                    continue;
                }
                prev[next] = Some(node);
                if self.goal_filter.contains(&next) {
                    return Some((prev, next));
                }
                que.push_back((next, Some(kind))).unwrap();
            }
        }
        None
    }

    pub fn shortest_path(
        &self,
        is_wall: impl Fn(&Coordinate<W>) -> bool,
    ) -> Option<Vec<Coordinate<W>, QUE_MAX>> {
        let (prev, mut cur) = self.bfs_tree(is_wall)?;
        let mut path = Vec::new();
        while let Some(next) = prev[cur] {
            path.push(cur).unwrap();
            if cur == self.start {
                break;
            }
            cur = next;
        }
        path.reverse();
        Some(path)
    }

    fn unchecked_walls(
        &self,
        wall_state: impl Fn(&Coordinate<W>) -> WallState,
        prev: &[Option<Coordinate<W>>],
        goal: Coordinate<W>,
    ) -> Option<Filter> {
        let mut filter = Filter::new();
        let mut flag = false;
        let mut cur = goal;
        while let Some(next) = prev[cur] {
            cur.intermediate_coords(&next)
                .into_iter()
                .filter(|node| wall_state(node) == WallState::Unchecked)
                .for_each(|node| {
                    flag = true;
                    filter.set(&node);
                });
            if next == self.start {
                break;
            }
            cur = next;
        }
        if flag {
            Some(filter)
        } else {
            None
        }
    }

    fn candidates(
        current: &Coordinate<W>,
        wall_state: impl Fn(&Coordinate<W>) -> WallState + Copy,
        unchecked_wall: impl Fn(&Coordinate<W>) -> bool,
    ) -> Option<Vec<Coordinate<W>, 6>> {
        let mut que = Deque::<_, QUE_MAX>::new();
        let mut is_visisted = [false; QUE_MAX];
        is_visisted[*current] = true;
        let neighbors = current.neighbors(wall_state);

        let candidates = |nearest: Coordinate<W>| {
            let mut candidates = Vec::new();
            candidates.push(nearest).unwrap();
            let is_near = |coord| nearest.is_neighbor(coord);
            let is_self = |coord| &nearest == coord || current == coord;
            neighbors
                .iter()
                .filter(|coord| is_near(coord) && !is_self(*coord))
                .for_each(|coord| candidates.push(*coord).unwrap());
            neighbors
                .iter()
                .filter(|coord| !is_near(coord) && !is_self(*coord))
                .for_each(|coord| candidates.push(*coord).unwrap());
            candidates
        };

        for (i, &next) in neighbors.iter().enumerate() {
            if unchecked_wall(&next) {
                return Some(candidates(next));
            }
            que.push_back((next, i)).unwrap();
            is_visisted[next] = true;
        }

        while let Some((node, i)) = que.pop_front() {
            for next in node.neighbors(wall_state) {
                if is_visisted[next] {
                    continue;
                }
                if unchecked_wall(&next) {
                    return Some(candidates(neighbors[i]));
                }
                is_visisted[next] = true;
                que.push_back((next, i)).unwrap();
            }
        }
        None
    }

    pub fn search(
        &self,
        current: &Coordinate<W>,
        wall_state: impl Fn(&Coordinate<W>) -> WallState + Copy,
    ) -> Result<Option<Commander<W>>, SearchError> {
        let (prev, goal) = self
            .bfs_tree(|coord| wall_state(coord) == WallState::Checked { exists: true })
            .ok_or(SearchError::Unreachable)?;

        let walls_filter = self.unchecked_walls(wall_state, &prev, goal);

        // All walls between the path from start to goal are checked.
        if walls_filter.is_none() {
            return Ok(None);
        }
        let walls_filter =
            walls_filter.expect("Should never fail: `walls` have already been verified as `Some`.");

        let candidates =
            Self::candidates(current, wall_state, |coord| walls_filter.contains(coord))
                .ok_or(SearchError::Unreachable)?;

        Ok(Some(Commander { candidates }))
    }
}

#[derive(Debug)]
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AbsoluteDirection {
    North,
    East,
    South,
    West,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum RelativeDirection {
    Front,
    Right,
    Left,
    Back,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct SearchState<const W: u8> {
    pub(crate) coord: Coordinate<W>,
    pub(crate) dir: AbsoluteDirection,
}

impl<const W: u8> SearchState<W> {
    pub fn new(coord: Coordinate<W>, dir: AbsoluteDirection) -> Option<Self> {
        match (coord.is_top, dir) {
            (true, AbsoluteDirection::North | AbsoluteDirection::South)
            | (false, AbsoluteDirection::East | AbsoluteDirection::West) => {
                Some(Self { coord, dir })
            }
            _ => None,
        }
    }

    pub fn coordinate(&self) -> &Coordinate<W> {
        &self.coord
    }

    pub fn direction(&self) -> &AbsoluteDirection {
        &self.dir
    }
}

impl<const W: u8> SearchState<W> {
    pub fn update(&mut self, next_coord: &Coordinate<W>) -> Option<RelativeDirection> {
        let SearchState {
            coord: cur_coord,
            dir: cur_dir,
        } = &self;
        let (abs, rel) = match (cur_coord.is_top, cur_dir) {
            (true, AbsoluteDirection::North) => match (
                next_coord.is_top,
                next_coord.x as i8 - cur_coord.x as i8,
                next_coord.y as i8 - cur_coord.y as i8,
            ) {
                (true, 0, 1) => (AbsoluteDirection::North, RelativeDirection::Front),
                (false, 0, 1) => (AbsoluteDirection::East, RelativeDirection::Right),
                (false, -1, 1) => (AbsoluteDirection::West, RelativeDirection::Left),
                (true, 0, -1) | (false, -1, 0) | (false, 0, 0) => {
                    (AbsoluteDirection::South, RelativeDirection::Back)
                }
                _ => return None,
            },
            (true, AbsoluteDirection::South) => match (
                next_coord.is_top,
                next_coord.x as i8 - cur_coord.x as i8,
                next_coord.y as i8 - cur_coord.y as i8,
            ) {
                (true, 0, -1) => (AbsoluteDirection::South, RelativeDirection::Front),
                (false, 0, 0) => (AbsoluteDirection::East, RelativeDirection::Left),
                (false, -1, 0) => (AbsoluteDirection::West, RelativeDirection::Right),
                (true, 0, 1) | (false, -1, 1) | (false, 0, 1) => {
                    (AbsoluteDirection::North, RelativeDirection::Back)
                }
                _ => return None,
            },
            (false, AbsoluteDirection::East) => match (
                next_coord.is_top,
                next_coord.x as i8 - cur_coord.x as i8,
                next_coord.y as i8 - cur_coord.y as i8,
            ) {
                (false, 1, 0) => (AbsoluteDirection::East, RelativeDirection::Front),
                (true, 1, 0) => (AbsoluteDirection::North, RelativeDirection::Left),
                (true, 1, -1) => (AbsoluteDirection::South, RelativeDirection::Right),
                (true, 0, 0) | (true, 0, -1) | (false, -1, 0) => {
                    (AbsoluteDirection::West, RelativeDirection::Back)
                }
                _ => return None,
            },
            (false, AbsoluteDirection::West) => match (
                next_coord.is_top,
                next_coord.x as i8 - cur_coord.x as i8,
                next_coord.y as i8 - cur_coord.y as i8,
            ) {
                (false, -1, 0) => (AbsoluteDirection::West, RelativeDirection::Front),
                (true, 0, -1) => (AbsoluteDirection::South, RelativeDirection::Left),
                (true, 0, 0) => (AbsoluteDirection::North, RelativeDirection::Right),
                (true, 1, 0) | (true, 1, -1) | (false, 1, 0) => {
                    (AbsoluteDirection::East, RelativeDirection::Back)
                }
                _ => return None,
            },
            _ => return None,
        };
        if rel != RelativeDirection::Back {
            self.coord = *next_coord;
        }
        self.dir = abs;
        Some(rel)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wall::Walls;
    use std::vec::Vec;

    fn new_coord<const W: u8>((x, y, is_top): (u8, u8, bool)) -> Coordinate<W> {
        Coordinate::new(x, y, is_top).unwrap()
    }

    #[test]
    fn test_shortest_path() {
        let test_cases = vec![(
            (0, 0, true),
            [(1, 0, true), (1, 0, false)],
            include_str!("../../mazes/maze4_1.dat"),
            vec![
                (0, 0, true),
                (0, 2, true),
                (0, 3, false),
                (2, 3, false),
                (3, 2, true),
                (3, 0, true),
                (1, 2, false),
                (1, 1, true),
                (1, 0, true),
            ],
        )];

        const W: u8 = 4;
        for (start, goals, walls, expected) in test_cases {
            let start = new_coord(start);
            let goals = goals.map(|goal| new_coord(goal));
            let walls = walls.parse::<Walls<W>>().unwrap();
            let expected = expected.into_iter().map(new_coord::<W>).collect::<Vec<_>>();
            let searcher = Searcher::<W>::new(start, &goals);
            let path = searcher
                .shortest_path(|coord| {
                    matches!(
                        walls.wall_state(coord),
                        WallState::Checked { exists: true } | WallState::Unchecked
                    )
                })
                .unwrap();
            assert_eq!(path.as_slice(), expected.as_slice());
        }
    }

    #[test]
    fn test_unchecked_wall() {
        let test_cases = vec![(
            (0, 0, true),
            [(1, 0, true), (1, 0, false)],
            include_str!("../../mazes/maze4_1.dat"),
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
        for (start, goals, walls, unchecked_walls, expected) in test_cases {
            let start = new_coord(start);
            let goals = goals.map(|goal| new_coord(goal));
            let mut walls = walls.parse::<Walls<W>>().unwrap();
            for wall in unchecked_walls {
                walls.update(&new_coord(wall), &WallState::Unchecked);
            }
            let expected = expected.into_iter().map(new_coord::<W>).collect::<Vec<_>>();
            let searcher = Searcher::<W>::new(start, &goals);
            let (prev, goal) = searcher
                .bfs_tree(|coord| {
                    matches!(walls.wall_state(coord), WallState::Checked { exists: true })
                })
                .unwrap();
            let walls_filter = searcher
                .unchecked_walls(|coord| walls.wall_state(coord), &prev, goal)
                .unwrap();
            assert!(expected
                .into_iter()
                .all(|coord| walls_filter.contains(&coord)));
        }
    }

    #[test]
    fn test_search() {
        let test_cases = vec![(
            (0, 0, true),
            [(1, 0, true), (1, 0, false)],
            include_str!("../../mazes/maze4_1.dat"),
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
        for (start, goals, walls, unchecked_walls, current, expected) in test_cases {
            let start = new_coord(start);
            let goals = goals.map(|goal| new_coord(goal));
            let mut walls = walls.parse::<Walls<W>>().unwrap();
            for wall in unchecked_walls {
                walls.update(&new_coord(wall), &WallState::Unchecked);
            }
            let current = new_coord(current);
            let expected = expected.into_iter().map(new_coord::<W>).collect::<Vec<_>>();
            let searcher = Searcher::<W>::new(start, &goals);
            let Commander { candidates } = searcher
                .search(&current, |coord| walls.wall_state(coord))
                .unwrap()
                .unwrap();
            assert_eq!(candidates.as_slice(), expected.as_slice());
        }
    }

    #[test]
    fn test_extended_neighbors_corner1() {
        new_coord::<4>((0, 0, true)).extended_neighbors(|_| true, None);
    }
}
