mod direction;
mod node;

use core::marker::PhantomData;
use core::ops::Mul;

use generic_array::{ArrayLength, GenericArray};
use heapless::{consts::*, Vec};
use typenum::{PowerOfTwo, Unsigned};

use crate::operator::{
    CheckableGraph, DirectionalGraph, Graph, GraphTranslator, PatternInstructor,
};
use crate::pattern::Pattern;
use direction::{AbsoluteDirection, RelativeDirection};
use node::{Location, Node, NodeId, Position};

struct WallPosition<N> {
    x: u16,
    y: u16,
    z: bool,
    _size: PhantomData<fn() -> N>,
}

impl<N> WallPosition<N>
where
    N: Unsigned + PowerOfTwo,
{
    fn new(x: u16, y: u16, z: bool) -> Self {
        Self {
            x,
            y,
            z,
            _size: PhantomData,
        }
    }

    #[inline]
    fn y_offset() -> u32 {
        N::USIZE.trailing_zeros()
    }

    #[inline]
    fn z_offset() -> u32 {
        2 * N::USIZE.trailing_zeros()
    }

    fn as_index(self) -> usize {
        self.x as usize
            | (self.y as usize) << Self::y_offset()
            | (self.z as usize) << Self::z_offset()
    }

    fn from_node(node: Node<N>) -> Option<Self> {
        Self::from_position(node.position())
    }

    fn from_position(position: Position<N>) -> Option<Self> {
        use Location::*;
        let x = position.x() / 2;
        let y = position.y() / 2;
        if x < 0 || y < 0 {
            return None;
        }
        let z = match position.location() {
            Cell => return None,
            HorizontalBound => false,
            VerticalBound => true,
        };
        Some(Self::new(x as u16, y as u16, z))
    }
}

pub struct Maze<N, F>
where
    N: Mul<N>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
    F: Fn(Pattern) -> u16,
{
    is_checked: GenericArray<bool, <<N as Mul<N>>::Output as Mul<U2>>::Output>,
    is_wall: GenericArray<bool, <<N as Mul<N>>::Output as Mul<U2>>::Output>,
    costs: F,
}

impl<N, F> Maze<N, F>
where
    N: Mul<N> + Unsigned + PowerOfTwo,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
    F: Fn(Pattern) -> u16,
    Node<N>: core::fmt::Debug,
{
    pub fn new(costs: F) -> Self {
        Self {
            is_checked: GenericArray::default(),
            is_wall: GenericArray::default(),
            costs,
        }
    }

    #[inline]
    fn is_wall(&self, position: WallPosition<N>) -> bool {
        self.is_wall[position.as_index()]
    }

    #[inline]
    fn is_wall_by_position(&self, position: Position<N>) -> bool {
        if let Some(wall_position) = WallPosition::from_position(position) {
            self.is_wall(wall_position)
        } else {
            true
        }
    }

    #[inline]
    fn is_checked(&self, position: WallPosition<N>) -> bool {
        self.is_checked[position.as_index()]
    }

    #[inline]
    fn cost(&self, pattern: Pattern) -> u16 {
        (self.costs)(pattern)
    }

    fn cell_successors(&self, node: Node<N>) -> Vec<(Node<N>, u16), U70> {
        use AbsoluteDirection::*;
        use Pattern::*;
        use RelativeDirection::*;

        let is_wall_relative = |x: i16, y: i16| -> bool {
            self.is_wall_by_position(node.relative_position(x, y, North).unwrap())
        };
        let relative_node = |x: i16, y: i16, direction: RelativeDirection| -> Node<N> {
            node.relative_node(x, y, direction, North).unwrap()
        };
        if is_wall_relative(0, 1) {
            return Vec::new();
        }

        let right_successors = || -> Vec<(Node<N>, u16), U70> {
            let mut succs = Vec::new();
            if is_wall_relative(1, 2) {
                return succs;
            }
            succs
                .push((relative_node(1, 2, FrontRight), self.cost(FastRun45)))
                .unwrap();
            succs
                .push((relative_node(2, 2, Right), self.cost(FastRun90)))
                .unwrap();

            if is_wall_relative(2, 1) {
                return succs;
            }
            succs
                .push((relative_node(2, 1, BackRight), self.cost(FastRun135)))
                .unwrap();
            succs
                .push((relative_node(2, 0, Back), self.cost(FastRun180)))
                .unwrap();
            succs
        };

        let left_successors = || -> Vec<(Node<N>, u16), U4> {
            let mut succs = Vec::new();
            if is_wall_relative(-1, 2) {
                return succs;
            }
            succs
                .push((relative_node(-1, 2, FrontRight), self.cost(FastRun45)))
                .unwrap();
            succs
                .push((relative_node(-2, 2, Right), self.cost(FastRun90)))
                .unwrap();

            if is_wall_relative(-2, 1) {
                return succs;
            }
            succs
                .push((relative_node(-2, 1, BackRight), self.cost(FastRun135)))
                .unwrap();
            succs
                .push((relative_node(-2, 0, Back), self.cost(FastRun180)))
                .unwrap();
            succs
        };

        let mut succs = right_successors();
        succs.extend_from_slice(&left_successors()).unwrap();

        succs
            .push((relative_node(0, 1, Front), self.cost(Straight(1))))
            .unwrap();
        succs
            .push((relative_node(0, 2, Front), self.cost(Straight(2))))
            .unwrap();
        for i in 1..N::I16 / 2 {
            if is_wall_relative(0, 2 * i + 1) {
                break;
            }
            succs
                .push((
                    relative_node(0, 2 * i + 1, Front),
                    self.cost(Straight((2 * i + 1) as u16)),
                ))
                .unwrap();
            succs
                .push((
                    relative_node(0, 2 * i + 2, Front),
                    self.cost(Straight((2 * i + 2) as u16)),
                ))
                .unwrap();
        }

        succs
    }

    fn bound_straight_successors(&self, node: Node<N>) -> Vec<(Node<N>, u16), U70> {
        use AbsoluteDirection::*;
        use Pattern::*;
        use RelativeDirection::*;

        let is_wall_relative = |x: i16, y: i16| -> bool {
            self.is_wall_by_position(node.relative_position(x, y, North).unwrap())
        };
        let relative_node = |x: i16, y: i16, direction: RelativeDirection| -> Node<N> {
            node.relative_node(x, y, direction, North).unwrap()
        };

        let mut succs = Vec::new();
        succs
            .push((relative_node(0, 0, Back), self.cost(SpinBack)))
            .unwrap();
        if !is_wall_relative(1, 1) {
            succs
                .push((relative_node(1, 1, Right), self.cost(Search90)))
                .unwrap();
        }
        if !is_wall_relative(-1, 1) {
            succs
                .push((relative_node(-1, 1, Left), self.cost(Search90)))
                .unwrap();
        }

        succs
            .push((relative_node(0, 1, Front), self.cost(Straight(1))))
            .unwrap();
        succs
            .push((relative_node(0, 2, Front), self.cost(Straight(2))))
            .unwrap();
        for i in 1..N::I16 / 2 {
            if is_wall_relative(0, 2 * i + 1) {
                break;
            }
            succs
                .push((
                    relative_node(0, 2 * i + 1, Front),
                    self.cost(Straight((2 * i + 1) as u16)),
                ))
                .unwrap();
            succs
                .push((
                    relative_node(0, 2 * i + 2, Front),
                    self.cost(Straight((2 * i + 2) as u16)),
                ))
                .unwrap();
        }

        succs
    }

    fn bound_diagonal_successors(&self, node: Node<N>) -> Vec<(Node<N>, u16), U70> {
        use AbsoluteDirection::*;
        use Pattern::*;
        use RelativeDirection::*;

        let is_wall_relative = |x: i16, y: i16| -> bool {
            self.is_wall_by_position(node.relative_position(x, y, NorthEast).unwrap())
        };
        let relative_node = |x: i16, y: i16, direction: RelativeDirection| -> Node<N> {
            node.relative_node(x, y, direction, NorthEast).unwrap()
        };

        let mut succs = Vec::new();
        if is_wall_relative(1, 1) {
            return succs;
        }
        succs
            .push((relative_node(1, 1, Front), self.cost(StraightDiagonal(1))))
            .unwrap();
        succs
            .push((relative_node(2, 1, FrontRight), self.cost(FastRun45)))
            .unwrap();

        if is_wall_relative(2, 0) {
            return succs;
        }
        succs
            .push((relative_node(2, 0, BackRight), self.cost(FastRun135)))
            .unwrap();
        succs
            .push((relative_node(2, 0, Right), self.cost(FastRunDiagonal90)))
            .unwrap();

        for i in 2..N::I16 * 2 {
            if is_wall_relative(i, i) {
                break;
            }
            succs
                .push((
                    relative_node(i, i, Front),
                    self.cost(StraightDiagonal((i) as u16)),
                ))
                .unwrap();
        }

        succs
    }
}

impl<N, F> Graph<NodeId<N>, u16> for Maze<N, F>
where
    N: Mul<N> + Unsigned + PowerOfTwo,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
    F: Fn(Pattern) -> u16,
    Node<N>: core::fmt::Debug,
{
    type Edges = Vec<(NodeId<N>, u16), U70>;

    fn successors(&self, node_id: NodeId<N>) -> Self::Edges {
        use AbsoluteDirection::*;
        use Location::*;

        let node = node_id.as_node();
        let node_successors: Vec<(Node<N>, u16), U70> = match node.location() {
            Cell => match node.direction() {
                North | East | South | West => self.cell_successors(node),
                _ => Vec::new(),
            },
            VerticalBound | HorizontalBound => match node.direction() {
                North | East | South | West => self.bound_straight_successors(node),
                _ => self.bound_diagonal_successors(node),
            },
        };
        node_successors
            .into_iter()
            .filter_map(|(node, cost)| node.to_node_id().map(|node_id| (node_id, cost)))
            .collect()
    }

    fn predecessors(&self, node: NodeId<N>) -> Self::Edges {
        let mut preds = Vec::new();
        preds
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_successors() {}
}
