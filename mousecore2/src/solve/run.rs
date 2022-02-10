use core::{num::NonZeroU16, ops::ControlFlow};

use heapless::{binary_heap::Max, BinaryHeap, Vec};
use num_traits::{Bounded, PrimInt, Saturating, Unsigned};

use crate::solve::search::Coordinate;
use crate::trajectory::slalom::{SlalomDirection, SlalomKind};
use crate::WIDTH;

const QUE_MAX: usize = WIDTH * WIDTH * 16;
const PATH_MAX: usize = WIDTH * WIDTH;
const NEIGHBOR_MAX: usize = 2 * WIDTH + 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AbsoluteDirection {
    North = 0,
    NorthEast = 1,
    East = 2,
    SouthEast = 3,
    South = 4,
    SouthWest = 5,
    West = 6,
    NorthWest = 7,
}

impl AbsoluteDirection {
    fn from_u8(val: u8) -> AbsoluteDirection {
        use AbsoluteDirection::*;

        match val {
            0 => North,
            1 => NorthEast,
            2 => East,
            3 => SouthEast,
            4 => South,
            5 => SouthWest,
            6 => West,
            7 => NorthWest,
            _ => unreachable!(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct RunCoord<const W: u8> {
    x: u8,
    y: u8,
}

impl<const W: u8> RunCoord<W> {
    fn new(x: u8, y: u8) -> Option<Self> {
        if !W.is_power_of_two() || x >= (W << 1) || y >= (W << 1) {
            return None;
        }
        Some(Self { x, y })
    }

    fn new_relative(&self, dx: i8, dy: i8) -> Option<Self> {
        let x = (self.x as i8).checked_add(dx)?;
        let y = (self.y as i8).checked_add(dy)?;
        if x < 0 || y < 0 {
            return None;
        }
        Self::new(x as u8, y as u8)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Node<const W: u8> {
    coord: RunCoord<W>,
    dir: AbsoluteDirection,
}

impl<const W: u8> Node<W> {
    pub fn new(x: u8, y: u8, dir: AbsoluteDirection) -> Option<Self> {
        use AbsoluteDirection::*;

        let coord = RunCoord::new(x, y)?;
        if matches!(
            (dir, (x & 1, y & 1)),
            (
                NorthEast | NorthWest | SouthEast | SouthWest,
                (1, 0) | (0, 1)
            ) | (North | East | South | West, (0, 0))
        ) {
            Some(Self { coord, dir })
        } else {
            None
        }
    }

    pub fn x(&self) -> u8 {
        self.coord.x
    }

    pub fn y(&self) -> u8 {
        self.coord.y
    }

    pub fn direction(&self) -> AbsoluteDirection {
        self.dir
    }

    fn as_index(&self) -> usize {
        use AbsoluteDirection::*;

        let dir = if (self.coord.x ^ self.coord.y) & 1 == 1 {
            match self.dir {
                NorthEast => 0,
                SouthEast => 1,
                SouthWest => 2,
                NorthWest => 3,
                _ => unreachable!(),
            }
        } else if (self.coord.x | self.coord.y) & 1 == 0 {
            match self.dir {
                North => 0,
                East => 1,
                South => 2,
                West => 3,
                _ => unreachable!(),
            }
        } else {
            unreachable!()
        };

        self.coord.x as usize
            | ((self.coord.y as usize) << Self::y_offset())
            | (dir << Self::dir_offset())
    }

    fn y_offset() -> u32 {
        W.trailing_zeros() + 1
    }

    fn dir_offset() -> u32 {
        (W.trailing_zeros() + 1) << 1
    }

    fn successors(
        &self,
        is_wall: impl Fn(&Coordinate<W>) -> bool,
    ) -> Vec<(Self, EdgeKind), NEIGHBOR_MAX> {
        let mut succs = Vec::new();
        self.succs_flow(
            |dx, dy| {
                if let Some(coord) = self.coord.new_relative(dx, dy) {
                    let coord = match (coord.x & 1, coord.y & 1) {
                        (1, 0) => Coordinate::new(coord.x >> 1, coord.y >> 1, false).unwrap(),
                        (0, 1) => Coordinate::new(coord.x >> 1, coord.y >> 1, true).unwrap(),
                        _ => unreachable!(),
                    };
                    if is_wall(&coord) {
                        ControlFlow::Break(())
                    } else {
                        ControlFlow::Continue(())
                    }
                } else {
                    ControlFlow::Break(())
                }
            },
            |dx, dy, dir, kind| {
                if let Some(node) = self.new_relative(dx, dy, dir) {
                    succs.push((node, kind)).unwrap();
                    ControlFlow::Continue(())
                } else {
                    ControlFlow::Break(())
                }
            },
        );
        succs
    }

    fn succs_flow(
        &self,
        cont: impl Fn(i8, i8) -> ControlFlow<(), ()>,
        mut add: impl FnMut(i8, i8, AbsoluteDirection, EdgeKind) -> ControlFlow<(), ()>,
    ) -> ControlFlow<(), ()> {
        use AbsoluteDirection::*;
        use EdgeKind::*;

        let rot = |dx: i8, dy: i8| match self.dir {
            North | NorthEast => (dx, dy),
            East | SouthEast => (dy, -dx),
            South | SouthWest => (-dx, -dy),
            West | NorthWest => (-dy, dx),
        };

        let cont = |dx: i8, dy: i8| {
            let (dx, dy) = rot(dx, dy);
            cont(dx, dy)
        };

        let mut add = |dx: i8, dy: i8, dir, kind| {
            let (dx, dy) = rot(dx, dy);
            let dir = match self.dir {
                North | NorthEast => dir,
                East | SouthEast => AbsoluteDirection::from_u8((dir as u8 + 2) & 7),
                South | SouthWest => AbsoluteDirection::from_u8((dir as u8 + 4) & 7),
                West | NorthWest => AbsoluteDirection::from_u8((dir as u8 + 6) & 7),
            };
            add(dx, dy, dir, kind)
        };

        match (self.coord.x & 1, self.coord.y & 1, self.dir) {
            (0, 0, North | East | South | West) => {
                cont(0, 1)?;
                // straight
                (1..).try_for_each(|i| {
                    cont(0, 2 * i - 1)?;
                    add(0, 2 * i, North, Straight(i as u8))
                });
                // right
                (|| {
                    cont(1, 2)?;
                    add(1, 2, NorthEast, Slalom45)?;
                    add(2, 2, East, Slalom90)?;
                    cont(2, 1)?;
                    add(2, 1, SouthEast, Slalom135)?;
                    add(2, 0, South, Slalom180)
                })();
                // left
                (|| {
                    cont(-1, 2)?;
                    add(-1, 2, NorthWest, Slalom45)?;
                    add(-2, 2, West, Slalom90)?;
                    cont(-2, 1)?;
                    add(-2, 1, SouthWest, Slalom135)?;
                    add(-2, 0, South, Slalom180)
                })();
            }
            // left
            (1, 0, NorthEast | SouthWest) | (0, 1, NorthWest | SouthEast) => {
                cont(1, 1)?;
                (1..).try_for_each(|i| {
                    cont(i, i)?;
                    add(i, i, NorthEast, StraightDiagonal(i as u8))
                });
                add(1, 2, North, Slalom45)?;
                cont(0, 2)?;
                add(0, 2, NorthWest, SlalomDiagonal90)?;
                add(-1, 2, West, Slalom135)?;
            }
            // right
            (0, 1, NorthEast | SouthWest) | (1, 0, NorthWest | SouthEast) => {
                cont(1, 1)?;
                (1..).try_for_each(|i| {
                    cont(i, i)?;
                    add(i, i, NorthEast, StraightDiagonal(i as u8))
                });
                add(2, 1, East, Slalom45)?;
                cont(2, 0)?;
                add(2, 0, SouthEast, SlalomDiagonal90)?;
                add(2, -1, South, Slalom135)?;
            }
            _ => unreachable!(),
        }
        ControlFlow::Break(())
    }

    fn new_relative(&self, dx: i8, dy: i8, dir: AbsoluteDirection) -> Option<Self> {
        let coord = self.coord.new_relative(dx, dy)?;
        Some(Self { coord, dir })
    }

    pub fn trajectory_kind(&self, other: &Self) -> Option<TrajectoryKind> {
        use AbsoluteDirection::*;
        use SlalomDirection::*;
        use SlalomKind::*;
        use TrajectoryKind::*;

        let dx = other.coord.x as i8 - self.coord.x as i8;
        let dy = other.coord.y as i8 - self.coord.y as i8;
        let rel = match self.dir {
            North | NorthEast => (dx, dy, other.dir),
            East | SouthEast => (
                -dy,
                dx,
                AbsoluteDirection::from_u8((6 + other.dir as u8) & 7),
            ),
            South | SouthWest => (
                -dx,
                -dy,
                AbsoluteDirection::from_u8((4 + other.dir as u8) & 7),
            ),
            West | NorthWest => (
                dy,
                -dx,
                AbsoluteDirection::from_u8((2 + other.dir as u8) & 7),
            ),
        };
        Some(match (self.coord.x & 1, self.coord.y & 1, self.dir) {
            (0, 0, North | East | South | West) => match rel {
                (0, y, North) if y > 0 && y & 1 == 0 => Straight((y as u8) >> 1),
                (1, 2, NorthEast) => Slalom(FastRun45, Right),
                (2, 2, East) => Slalom(FastRun90, Right),
                (2, 1, SouthEast) => Slalom(FastRun135, Right),
                (2, 0, South) => Slalom(FastRun180, Right),
                (-1, 2, NorthEast) => Slalom(FastRun45, Left),
                (-2, 2, East) => Slalom(FastRun90, Left),
                (-2, 1, SouthEast) => Slalom(FastRun135, Left),
                (-2, 0, South) => Slalom(FastRun180, Left),
                _ => return None,
            },
            //left
            (1, 0, NorthEast | SouthWest) | (0, 1, NorthWest | SouthEast) => match rel {
                (x, y, NorthEast) if x == y && x > 0 => StraightDiagonal(x as u8),
                (1, 2, North) => Slalom(FastRun45Rev, Left),
                (0, 2, NorthWest) => Slalom(FastRunDiagonal90, Left),
                (-1, 2, West) => Slalom(FastRun135Rev, Left),
                _ => return None,
            },
            // right
            (0, 1, NorthEast | SouthWest) | (1, 0, NorthWest | SouthEast) => match rel {
                (x, y, NorthEast) if x == y && x > 0 => StraightDiagonal(x as u8),
                (2, 1, East) => Slalom(FastRun45Rev, Right),
                (2, 0, SouthEast) => Slalom(FastRunDiagonal90, Right),
                (2, -1, South) => Slalom(FastRun135Rev, Right),
                _ => return None,
            },
            _ => return None,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrajectoryKind {
    Straight(u8),
    StraightDiagonal(u8),
    Slalom(SlalomKind, SlalomDirection),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum EdgeKind {
    Straight(u8),
    StraightDiagonal(u8),
    Slalom45,
    Slalom90,
    Slalom135,
    Slalom180,
    SlalomDiagonal90,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, PartialOrd, Ord)]
struct NodeId<const W: u8>(NonZeroU16);

impl<const W: u8> NodeId<W> {
    fn as_index(&self) -> usize {
        self.0.get() as usize >> 1
    }
}

impl<const W: u8> From<Node<W>> for NodeId<W> {
    fn from(value: Node<W>) -> Self {
        NodeId(NonZeroU16::new(((value.as_index() as u16) << 1) | 1).unwrap())
    }
}

impl<const W: u8> From<NodeId<W>> for Node<W> {
    fn from(value: NodeId<W>) -> Self {
        use AbsoluteDirection::*;

        let value = value.0.get();
        let x = ((value >> 1) & ((1 << Self::y_offset()) - 1)) as u8;
        let y = ((value & ((1 << (Self::dir_offset() + 1)) - 1)) >> (1 + Self::y_offset())) as u8;
        let dir = value >> (1 + Self::dir_offset());
        let dir = if (x ^ y) & 1 == 1 {
            match dir {
                0 => NorthEast,
                1 => SouthEast,
                2 => SouthWest,
                3 => NorthWest,
                _ => unreachable!(),
            }
        } else if (x | y) & 1 == 0 {
            match dir {
                0 => North,
                1 => East,
                2 => South,
                3 => West,
                _ => unreachable!(),
            }
        } else {
            unreachable!()
        };

        Self {
            coord: RunCoord { x, y },
            dir,
        }
    }
}

pub fn shortest_path<C, const W: u8>(
    start: Node<W>,
    is_goal: impl Fn(&Node<W>) -> bool,
    is_wall: impl Fn(&Coordinate<W>) -> bool + Copy,
    into_cost: impl Fn(&EdgeKind) -> C,
) -> Option<Vec<Node<W>, PATH_MAX>>
where
    C: Bounded + PrimInt + Saturating + Unsigned,
{
    struct CostNode<T, U> {
        cost: T,
        node: U,
    }

    impl<T: PartialEq, U> PartialEq for CostNode<T, U> {
        fn eq(&self, other: &Self) -> bool {
            self.cost.eq(&other.cost)
        }
    }

    impl<T: Eq, U> Eq for CostNode<T, U> {}

    impl<T: Ord, U> Ord for CostNode<T, U> {
        fn cmp(&self, other: &Self) -> core::cmp::Ordering {
            self.cost.cmp(&other.cost)
        }
    }

    impl<T: PartialOrd, U> PartialOrd for CostNode<T, U> {
        fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
            self.cost.partial_cmp(&other.cost)
        }
    }

    impl<T, const W: u8> core::ops::Index<NodeId<W>> for [T] {
        type Output = T;

        fn index(&self, index: NodeId<W>) -> &Self::Output {
            &(self)[index.as_index()]
        }
    }

    impl<T, const W: u8> core::ops::IndexMut<NodeId<W>> for [T] {
        fn index_mut(&mut self, index: NodeId<W>) -> &mut Self::Output {
            &mut (self)[index.as_index()]
        }
    }

    let start = NodeId::from(start);

    let mut dist = [C::max_value(); QUE_MAX];
    let mut prev = [None; QUE_MAX];
    let mut heap = BinaryHeap::<_, Max, QUE_MAX>::new();
    dist[start] = C::zero();
    prev[start] = Some(start);
    heap.push(CostNode {
        cost: C::zero(),
        node: start,
    })
    .ok();

    let mut goal = None;

    while let Some(CostNode { cost, node }) = heap.pop() {
        if dist[node] < cost {
            continue;
        }

        let nodet = Node::from(node);
        if is_goal(&nodet) {
            goal = Some(node);
            break;
        }

        for (next, kind) in nodet.successors(is_wall) {
            let next = NodeId::from(next);
            let cost = cost.saturating_add(into_cost(&kind));
            heap.push(CostNode { cost, node: next }).ok();
            dist[next] = cost;
            prev[next] = Some(node);
        }
    }

    let goal = goal?;

    let mut cur = goal;
    let mut path = Vec::new();
    while let Some(next) = prev[cur] {
        path.push(Node::from(cur)).unwrap();
        if cur == start {
            break;
        }
        cur = next;
    }
    path.reverse();
    Some(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{solve::search::WallState, wall::Walls};
    use AbsoluteDirection::*;

    fn test_shortest_path<const W: u8>(
        walls: &str,
        goals: &[(u8, u8, AbsoluteDirection)],
        expected: &[(u8, u8, AbsoluteDirection)],
    ) {
        use EdgeKind::*;

        let walls = walls.parse::<Walls<W>>().unwrap();
        let start = new_node((0, 0, AbsoluteDirection::North));
        let goals = goals
            .into_iter()
            .map(|&node| new_node(node))
            .collect::<Vec<_, 4>>();
        let expected = expected
            .into_iter()
            .map(|&node| new_node::<W>(node))
            .collect::<Vec<_, PATH_MAX>>();

        let path = shortest_path(
            start,
            |node| goals.iter().any(|goal| node == goal),
            |coord| {
                matches!(
                    walls.wall_state(&coord),
                    WallState::Checked { exists: true } | WallState::Unchecked
                )
            },
            |kind| match kind {
                Straight(x) => *x as u16 * 10,
                StraightDiagonal(x) => *x as u16 * 7,
                Slalom45 => 12,
                Slalom90 => 15,
                Slalom135 => 20,
                Slalom180 => 25,
                SlalomDiagonal90 => 15,
            },
        );
        assert_eq!(path.unwrap(), expected);
    }

    fn new_node<const W: u8>((x, y, dir): (u8, u8, AbsoluteDirection)) -> Node<W> {
        Node::new(x, y, dir).unwrap()
    }

    #[test]
    fn test_shortest_path1() {
        test_shortest_path::<4>(
            include_str!("../../mazes/empty4.dat"),
            &[(2, 0, South)],
            &[(0, 0, North), (2, 0, South)],
        )
    }

    #[test]
    fn test_shortest_path2() {
        test_shortest_path::<4>(
            include_str!("../../mazes/maze4_1.dat"),
            &[(2, 0, South)],
            &[
                (0, 0, North),
                (0, 4, North),
                (2, 6, East),
                (4, 6, East),
                (6, 4, South),
                (4, 3, NorthWest),
                (2, 2, South),
                (2, 0, South),
            ],
        )
    }

    #[test]
    fn test_shortest_path3() {
        test_shortest_path::<4>(
            include_str!("../../mazes/maze4_2.dat"),
            &[(2, 0, South), (2, 0, West)],
            &[
                (0, 0, North),
                (0, 4, North),
                (2, 6, East),
                (4, 6, East),
                (6, 4, South),
                (6, 2, South),
                (4, 0, West),
                (2, 0, West),
            ],
        )
    }

    #[test]
    fn test_shortest_path4() {
        test_shortest_path::<4>(
            include_str!("../../mazes/maze4_3.dat"),
            &[(2, 0, South), (2, 0, West)],
            &[
                (0, 0, North),
                (1, 2, NorthEast),
                (4, 5, NorthEast),
                (6, 4, South),
                (6, 2, South),
                (4, 0, West),
                (2, 0, West),
            ],
        )
    }
}
