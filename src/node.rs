use core::marker::PhantomData;

use heapless::Vec;
use typenum::{consts::*, PowerOfTwo, Unsigned};

use crate::data_types::{AbsoluteDirection, RelativeDirection};
use crate::simple_maze::{GraphNode, WallNode, WallSpaceNode};
use crate::utils::forced_vec::ForcedVec;
use crate::wall_manager::Wall;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Pattern {
    Straight(u16),
    StraightDiagonal(u16),
    Search90,
    FastRun45,
    FastRun90,
    FastRun135,
    FastRun180,
    FastRunDiagonal90,
    SpinBack,
}

#[derive(Clone, PartialEq, Eq, Debug)]
struct Node<N> {
    x: i16,
    y: i16,
    direction: AbsoluteDirection,
    cost: fn(Pattern) -> u16,
    _maze_width: PhantomData<fn() -> N>,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct NodeCreationError {
    x: i16,
    y: i16,
    direction: AbsoluteDirection,
}

#[derive(Clone, PartialEq, Eq, Debug)]
struct RelativeWallError {
    x: i16,
    y: i16,
    direction: AbsoluteDirection,
    dx: i16,
    dy: i16,
    base_dir: AbsoluteDirection,
}

impl<N> Node<N> {
    unsafe fn new_unchecked(
        x: i16,
        y: i16,
        direction: AbsoluteDirection,
        cost: fn(Pattern) -> u16,
    ) -> Self {
        Self {
            x,
            y,
            direction,
            cost,
            _maze_width: PhantomData,
        }
    }
}

impl<N> Node<N>
where
    N: Unsigned,
{
    fn new(
        x: i16,
        y: i16,
        direction: AbsoluteDirection,
        cost: fn(Pattern) -> u16,
    ) -> Result<Self, NodeCreationError> {
        if x < 0 || y < 0 || x > Self::max() || y > Self::max() {
            Err(NodeCreationError { x, y, direction })
        } else {
            Ok(unsafe { Self::new_unchecked(x, y, direction, cost) })
        }
    }

    fn max() -> i16 {
        2 * N::I16 - 1
    }

    fn relative(
        &self,
        dx: i16,
        dy: i16,
        ddir: RelativeDirection,
        base_dir: AbsoluteDirection,
    ) -> Result<Self, NodeCreationError> {
        use RelativeDirection::*;

        let relative_dir = base_dir.relative(self.direction);
        let (dx, dy) = match relative_dir {
            Front => (dx, dy),
            Right => (dy, -dx),
            Back => (-dx, -dy),
            Left => (-dy, dx),
            _ => unreachable!(),
        };
        Self::new(
            self.x + dx,
            self.y + dy,
            self.direction.rotate(ddir),
            self.cost,
        )
    }
}

impl<N> Node<N>
where
    N: Unsigned + PowerOfTwo,
{
    fn relative_wall(
        &self,
        dx: i16,
        dy: i16,
        base_dir: AbsoluteDirection,
    ) -> Result<Wall<N>, RelativeWallError> {
        use RelativeDirection::*;

        let create_error = || RelativeWallError {
            x: self.x,
            y: self.y,
            direction: self.direction,
            dx,
            dy,
            base_dir,
        };

        let relative_dir = base_dir.relative(self.direction);
        let (dx, dy) = match relative_dir {
            Front => (dx, dy),
            Right => (dy, -dx),
            Back => (-dx, -dy),
            Left => (-dy, dx),
            _ => unreachable!(),
        };
        let x = self.x + dx;
        let y = self.y + dy;
        if x < 0 || y < 0 {
            return Err(create_error());
        }

        let x = x as u16;
        let y = y as u16;
        if (x ^ y) & 1 == 0 {
            //not on a wall
            return Err(create_error());
        } else if x & 1 == 1 {
            //on a vertical wall
            Wall::<N>::new(x / 2, y / 2, false)
        } else {
            //on a horizontal wall
            Wall::<N>::new(x / 2, y / 2, true)
        }
        .map_err(|_| create_error())
    }
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub struct SearchNode<N>(Node<N>);

impl<N> SearchNode<N> {
    pub unsafe fn new_unchecked(
        x: i16,
        y: i16,
        direction: AbsoluteDirection,
        cost: fn(Pattern) -> u16,
    ) -> Self {
        Self(Node::<N>::new_unchecked(x, y, direction, cost))
    }

    #[inline]
    fn is_valid_direction(x: i16, y: i16, direction: AbsoluteDirection) -> bool {
        use AbsoluteDirection::*;

        if (x ^ y) & 1 == 0 {
            false
        } else if x & 1 == 1 {
            match direction {
                East | West => true,
                _ => false,
            }
        } else {
            match direction {
                North | South => true,
                _ => false,
            }
        }
    }
}

impl<N> SearchNode<N>
where
    N: Unsigned,
{
    pub fn new(
        x: i16,
        y: i16,
        direction: AbsoluteDirection,
        cost: fn(Pattern) -> u16,
    ) -> Result<Self, NodeCreationError> {
        use core::convert::TryInto;
        Node::<N>::new(x, y, direction, cost)?.try_into()
    }

    fn relative(
        &self,
        dx: i16,
        dy: i16,
        ddir: RelativeDirection,
        base_dir: AbsoluteDirection,
    ) -> Result<Self, NodeCreationError> {
        use core::convert::TryInto;
        self.0.relative(dx, dy, ddir, base_dir)?.try_into()
    }
}

impl<N> SearchNode<N>
where
    N: Unsigned + PowerOfTwo,
{
    fn neighbors(&self, succs: bool) -> <Self as GraphNode>::WallNodesList {
        use Pattern::*;
        use RelativeDirection::*;

        let mut list = ForcedVec::new();
        let mut update = |dx: i16, dy: i16, ddir: RelativeDirection, pattern: Pattern| {
            use AbsoluteDirection::*;
            let (dx, dy) = if succs { (dx, dy) } else { (-dx, -dy) };
            if let Ok(wall) = self.0.relative_wall(dx, dy, North) {
                let mut wall_nodes = ForcedVec::new();
                wall_nodes.push(WallNode::Wall(wall));
                wall_nodes.push(WallNode::Node((
                    self.relative(dx, dy, ddir, North)
                        .expect("Should never panic!"),
                    (self.0.cost)(pattern),
                )));
                list.push(wall_nodes.into());
            }
        };

        update(1, 1, Right, Search90);
        update(0, 2, Front, Straight(1));
        update(-1, 1, Left, Search90);
        update(0, 0, Back, SpinBack);

        list.into()
    }
}

impl<N: Unsigned> core::convert::TryFrom<Node<N>> for SearchNode<N> {
    type Error = NodeCreationError;

    fn try_from(value: Node<N>) -> Result<Self, Self::Error> {
        if Self::is_valid_direction(value.x, value.y, value.direction) {
            Ok(Self(value))
        } else {
            Err(NodeCreationError {
                x: value.x,
                y: value.y,
                direction: value.direction,
            })
        }
    }
}

impl<N> WallSpaceNode for SearchNode<N> {
    type Wall = Wall<N>;
}

impl<N> GraphNode for SearchNode<N>
where
    N: Unsigned + PowerOfTwo,
{
    type Cost = u16;
    type WallNodes = Vec<WallNode<Self::Wall, (Self, Self::Cost)>, U2>;
    type WallNodesList = Vec<Self::WallNodes, U4>;

    fn successors(&self) -> Self::WallNodesList {
        self.neighbors(true)
    }

    fn predecessors(&self) -> Self::WallNodesList {
        self.neighbors(false)
    }
}

#[cfg(test)]
mod tests {
    use typenum::consts::*;

    use super::*;

    #[test]
    fn test_search_node_new() {
        use AbsoluteDirection::*;

        fn cost(_pattern: Pattern) -> u16 {
            unimplemented!()
        }

        let test_cases = vec![
            (
                1,
                1,
                North,
                Err(NodeCreationError {
                    x: 1,
                    y: 1,
                    direction: North,
                }),
            ),
            (
                0,
                0,
                North,
                Err(NodeCreationError {
                    x: 0,
                    y: 0,
                    direction: North,
                }),
            ),
            (
                0,
                1,
                North,
                Ok(SearchNode(Node {
                    x: 0,
                    y: 1,
                    direction: North,
                    cost,
                    _maze_width: PhantomData,
                })),
            ),
            (
                0,
                1,
                South,
                Ok(SearchNode(Node {
                    x: 0,
                    y: 1,
                    direction: South,
                    cost,
                    _maze_width: PhantomData,
                })),
            ),
            (
                0,
                1,
                East,
                Err(NodeCreationError {
                    x: 0,
                    y: 1,
                    direction: East,
                }),
            ),
            (
                1,
                0,
                East,
                Ok(SearchNode(Node {
                    x: 1,
                    y: 0,
                    direction: East,
                    cost,
                    _maze_width: PhantomData,
                })),
            ),
            (
                1,
                0,
                South,
                Err(NodeCreationError {
                    x: 1,
                    y: 0,
                    direction: South,
                }),
            ),
        ];

        for (x, y, dir, expected) in test_cases {
            assert_eq!(SearchNode::<U4>::new(x, y, dir, cost), expected);
        }
    }

    #[test]
    fn test_graph_search_node() {
        use AbsoluteDirection::*;

        fn cost(pattern: Pattern) -> u16 {
            use Pattern::*;

            match pattern {
                Search90 => 1,
                Straight(1) => 2,
                SpinBack => 3,
                _ => unreachable!(),
            }
        }

        let test_cases = vec![
            (
                (0, 1, North),
                vec![
                    vec![
                        WallNode::Wall((0, 1, false)),
                        WallNode::Node((1, 2, East, 1)),
                    ],
                    vec![
                        WallNode::Wall((0, 1, true)),
                        WallNode::Node((0, 3, North, 2)),
                    ],
                    vec![
                        WallNode::Wall((0, 0, true)),
                        WallNode::Node((0, 1, South, 3)),
                    ],
                ],
            ),
            (
                (1, 2, East),
                vec![
                    vec![
                        WallNode::Wall((1, 0, true)),
                        WallNode::Node((2, 1, South, 1)),
                    ],
                    vec![
                        WallNode::Wall((1, 1, false)),
                        WallNode::Node((3, 2, East, 2)),
                    ],
                    vec![
                        WallNode::Wall((1, 1, true)),
                        WallNode::Node((2, 3, North, 1)),
                    ],
                    vec![
                        WallNode::Wall((0, 1, false)),
                        WallNode::Node((1, 2, West, 3)),
                    ],
                ],
            ),
        ];

        type List = Vec<Vec<WallNode<Wall<U4>, (SearchNode<U4>, u16)>>>;

        use std::vec::Vec;
        for ((x, y, dir), expected) in test_cases {
            let node = SearchNode::<U4>::new(x, y, dir, cost).unwrap();
            let successors: List = node
                .successors()
                .into_iter()
                .map(|e| e.into_iter().collect())
                .collect();

            let expected: List = expected
                .into_iter()
                .map(|e| {
                    e.into_iter()
                        .map(|wall_node| match wall_node {
                            WallNode::Wall((x, y, top)) => {
                                WallNode::Wall(Wall::<U4>::new(x, y, top).unwrap())
                            }
                            WallNode::Node((x, y, dir, c)) => {
                                WallNode::Node((SearchNode::<U4>::new(x, y, dir, cost).unwrap(), c))
                            }
                        })
                        .collect()
                })
                .collect();
            assert_eq!(successors, expected, "{:?}", (x, y, dir));
        }
    }
}
