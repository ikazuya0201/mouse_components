mod direction;
mod node;

use core::marker::PhantomData;
use core::ops::{Add, Mul};

use generic_array::{ArrayLength, GenericArray};
use heapless::{consts::*, Vec};
use typenum::{PowerOfTwo, Unsigned};

use crate::operator::{Graph, ReducedGraph};
use crate::pattern::Pattern;
use direction::{AbsoluteDirection, RelativeDirection};
use node::{Location, Node, NodeId, Position};

#[derive(Clone)]
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
    fn max() -> u16 {
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
        let mut maze = Self {
            is_checked: GenericArray::default(),
            is_wall: GenericArray::default(),
            costs,
        };
        maze.initialize();
        maze
    }

    pub fn initialize(&mut self) {
        for i in 0..WallPosition::<N>::max() + 1 {
            self.check_wall(WallPosition::new(i, WallPosition::<N>::max(), false), true);
            self.check_wall(WallPosition::new(WallPosition::<N>::max(), i, true), true);
        }
    }

    fn check_wall(&mut self, position: WallPosition<N>, is_wall: bool) {
        self.update_wall(position.clone(), is_wall);
        self.update_checked(position, true);
    }

    fn update_wall(&mut self, position: WallPosition<N>, is_wall: bool) {
        self.is_wall[position.as_index()] = is_wall;
    }

    fn update_checked(&mut self, position: WallPosition<N>, is_checked: bool) {
        self.is_checked[position.as_index()] = is_checked;
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

    fn is_wall_relative_fn<'a>(
        &'a self,
        node: &'a Node<N>,
        base_dir: AbsoluteDirection,
        is_successors: bool,
    ) -> impl Fn(i16, i16) -> bool + 'a {
        move |x: i16, y: i16| -> bool {
            if is_successors {
                self.is_wall_by_position(node.relative_position(x, y, base_dir).unwrap())
            } else {
                self.is_wall_by_position(node.relative_position(-x, -y, base_dir).unwrap())
            }
        }
    }

    fn relative_node_fn<'a>(
        node: &'a Node<N>,
        base_dir: AbsoluteDirection,
        is_successors: bool,
    ) -> impl Fn(i16, i16, RelativeDirection) -> Node<N> + 'a {
        move |x: i16, y: i16, direction: RelativeDirection| -> Node<N> {
            if is_successors {
                node.relative_node(x, y, direction, base_dir).unwrap()
            } else {
                node.relative_node(-x, -y, direction, base_dir).unwrap()
            }
        }
    }
}

impl<N, F> Maze<N, F>
where
    N: Mul<N> + Mul<U2> + Unsigned + PowerOfTwo,
    <N as Mul<U2>>::Output: Add<U10>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(Node<N>, u16)>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
    F: Fn(Pattern) -> u16,
    Node<N>: core::fmt::Debug,
{
    fn cell_neighbors(
        &self,
        node: Node<N>,
        is_successors: bool,
    ) -> Vec<(Node<N>, u16), <<N as Mul<U2>>::Output as Add<U10>>::Output> {
        use AbsoluteDirection::*;
        use Pattern::*;
        use RelativeDirection::*;

        let is_wall_relative = self.is_wall_relative_fn(&node, North, is_successors);
        let relative_node = Self::relative_node_fn(&node, North, is_successors);

        if is_wall_relative(0, 1) {
            return Vec::new();
        }

        let right_successors =
            || -> Vec<(Node<N>, u16), <<N as Mul<U2>>::Output as Add<U10>>::Output> {
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
                .push((relative_node(-1, 2, FrontLeft), self.cost(FastRun45)))
                .unwrap();
            succs
                .push((relative_node(-2, 2, Left), self.cost(FastRun90)))
                .unwrap();

            if is_wall_relative(-2, 1) {
                return succs;
            }
            succs
                .push((relative_node(-2, 1, BackLeft), self.cost(FastRun135)))
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
        for i in 1..N::I16 {
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

    fn bound_straight_neighbors(
        &self,
        node: Node<N>,
        is_successors: bool,
    ) -> Vec<(Node<N>, u16), <<N as Mul<U2>>::Output as Add<U10>>::Output> {
        use AbsoluteDirection::*;
        use Pattern::*;
        use RelativeDirection::*;

        let is_wall_relative = self.is_wall_relative_fn(&node, North, is_successors);
        let relative_node = Self::relative_node_fn(&node, North, is_successors);

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
        for i in 1..N::I16 {
            if is_wall_relative(0, 2 * i) {
                break;
            }
            succs
                .push((
                    relative_node(0, 2 * i, Front),
                    self.cost(Straight((2 * i) as u16)),
                ))
                .unwrap();
            succs
                .push((
                    relative_node(0, 2 * i + 1, Front),
                    self.cost(Straight((2 * i + 1) as u16)),
                ))
                .unwrap();
        }

        succs
    }

    fn horizontal_bound_diagonal_neighbors(
        &self,
        node: Node<N>,
        is_successors: bool,
    ) -> Vec<(Node<N>, u16), <<N as Mul<U2>>::Output as Add<U10>>::Output> {
        use AbsoluteDirection::*;
        use Pattern::*;
        use RelativeDirection::*;

        let is_wall_relative = self.is_wall_relative_fn(&node, NorthEast, is_successors);
        let relative_node = Self::relative_node_fn(&node, NorthEast, is_successors);

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
                    self.cost(StraightDiagonal(i as u16)),
                ))
                .unwrap();
        }

        succs
    }

    fn vertical_bound_diagonal_neighbors(
        &self,
        node: Node<N>,
        is_successors: bool,
    ) -> Vec<(Node<N>, u16), <<N as Mul<U2>>::Output as Add<U10>>::Output> {
        use AbsoluteDirection::*;
        use Pattern::*;
        use RelativeDirection::*;

        let is_wall_relative = self.is_wall_relative_fn(&node, NorthEast, is_successors);
        let relative_node = Self::relative_node_fn(&node, NorthEast, is_successors);

        let mut succs = Vec::new();
        if is_wall_relative(1, 1) {
            return succs;
        }
        succs
            .push((relative_node(1, 1, Front), self.cost(StraightDiagonal(1))))
            .unwrap();
        succs
            .push((relative_node(1, 2, FrontLeft), self.cost(FastRun45)))
            .unwrap();

        if is_wall_relative(0, 2) {
            return succs;
        }
        succs
            .push((relative_node(0, 2, BackLeft), self.cost(FastRun135)))
            .unwrap();
        succs
            .push((relative_node(0, 2, Left), self.cost(FastRunDiagonal90)))
            .unwrap();

        for i in 2..N::I16 * 2 {
            if is_wall_relative(i, i) {
                break;
            }
            succs
                .push((
                    relative_node(i, i, Front),
                    self.cost(StraightDiagonal(i as u16)),
                ))
                .unwrap();
        }

        succs
    }
}

impl<N, F> Maze<N, F>
where
    N: Mul<N> + Mul<U2> + Unsigned + PowerOfTwo,
    <N as Mul<U2>>::Output: Add<U10>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(Node<N>, u16)>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(NodeId<N>, u16)>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
    F: Fn(Pattern) -> u16,
    Node<N>: core::fmt::Debug,
{
    fn neighbors(
        &self,
        node_id: NodeId<N>,
        is_successors: bool,
    ) -> <Self as Graph<NodeId<N>, u16>>::Edges {
        use AbsoluteDirection::*;
        use Location::*;

        let node = node_id.as_node();
        let node_neighbors = match node.location() {
            Cell => match node.direction() {
                North | East | South | West => self.cell_neighbors(node, is_successors),
                _ => Vec::new(),
            },
            VerticalBound => match node.direction() {
                East | West => self.bound_straight_neighbors(node, is_successors),
                NorthEast | SouthEast | SouthWest | NorthWest => {
                    self.vertical_bound_diagonal_neighbors(node, is_successors)
                }
                _ => Vec::new(),
            },
            HorizontalBound => match node.direction() {
                North | South => self.bound_straight_neighbors(node, is_successors),
                NorthEast | SouthEast | SouthWest | NorthWest => {
                    self.horizontal_bound_diagonal_neighbors(node, is_successors)
                }
                _ => Vec::new(),
            },
        };
        node_neighbors
            .into_iter()
            .filter_map(|(node, cost)| node.to_node_id().map(|node_id| (node_id, cost)))
            .collect()
    }

    fn is_on_reduced_graph(node: &Node<N>) -> bool {
        use AbsoluteDirection::*;
        use Location::*;

        match node.location() {
            VerticalBound => match node.direction() {
                East | West => true,
                _ => false,
            },
            HorizontalBound => match node.direction() {
                North | South => true,
                _ => false,
            },
            Cell => false,
        }
    }

    fn reduced_neighbors(
        &self,
        node_id: NodeId<N>,
        is_successors: bool,
    ) -> <Self as Graph<NodeId<N>, u16>>::Edges {
        use AbsoluteDirection::*;
        use Pattern::*;
        use RelativeDirection::*;

        let node = node_id.as_node();
        debug_assert!(Self::is_on_reduced_graph(&node));

        let is_wall_relative = self.is_wall_relative_fn(&node, North, is_successors);
        let relative_node = Self::relative_node_fn(&node, North, is_successors);

        let mut neighbors = Vec::<(Node<N>, u16), U4>::new();
        neighbors
            .push((relative_node(0, 0, Back), self.cost(SpinBack)))
            .unwrap();
        if !is_wall_relative(1, 1) {
            neighbors
                .push((relative_node(1, 1, Left), self.cost(Search90)))
                .unwrap();
        }
        if !is_wall_relative(-1, 1) {
            neighbors
                .push((relative_node(-1, 1, Right), self.cost(Search90)))
                .unwrap();
        }
        if !is_wall_relative(0, 2) {
            neighbors
                .push((relative_node(0, 2, Front), self.cost(Straight(2))))
                .unwrap();
        }
        neighbors
            .into_iter()
            .filter_map(|(node, cost)| node.to_node_id().map(|node_id| (node_id, cost)))
            .collect()
    }
}

impl<N, F> Graph<NodeId<N>, u16> for Maze<N, F>
where
    N: Mul<N> + Mul<U2> + Unsigned + PowerOfTwo,
    <N as Mul<U2>>::Output: Add<U10>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(Node<N>, u16)>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(NodeId<N>, u16)>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
    F: Fn(Pattern) -> u16,
    Node<N>: core::fmt::Debug,
{
    type Edges = Vec<(NodeId<N>, u16), <<N as Mul<U2>>::Output as Add<U10>>::Output>;

    fn successors(&self, node_id: NodeId<N>) -> Self::Edges {
        self.neighbors(node_id, true)
    }

    fn predecessors(&self, node_id: NodeId<N>) -> Self::Edges {
        self.neighbors(node_id, false)
    }
}

impl<N, F> ReducedGraph<NodeId<N>, u16> for Maze<N, F>
where
    N: Mul<N> + Mul<U2> + Unsigned + PowerOfTwo,
    <N as Mul<U2>>::Output: Add<U10>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(Node<N>, u16)>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(NodeId<N>, u16)>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
    F: Fn(Pattern) -> u16,
    Node<N>: core::fmt::Debug,
{
    fn reduced_successors(&self, node: NodeId<N>) -> Self::Edges {
        self.reduced_neighbors(node, true)
    }

    fn reduced_predecessors(&self, node: NodeId<N>) -> Self::Edges {
        self.reduced_neighbors(node, false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cost(pattern: Pattern) -> u16 {
        use Pattern::*;

        match pattern {
            Straight(length) => length,
            StraightDiagonal(length) => length,
            Search90 => 3,
            FastRun45 => 4,
            FastRun90 => 5,
            FastRun135 => 6,
            FastRun180 => 7,
            FastRunDiagonal90 => 8,
            SpinBack => 9,
        }
    }

    #[test]
    fn test_successors() {
        use AbsoluteDirection::*;
        use Pattern::*;

        let maze = Maze::<U4, _>::new(cost);

        let new = |x: u16, y: u16, direction: AbsoluteDirection| -> NodeId<U4> {
            NodeId::<U4>::new(x, y, direction)
        };

        let test_data = vec![
            (
                new(2, 0, North),
                vec![
                    (new(2, 1, North), cost(Straight(1))),
                    (new(2, 2, North), cost(Straight(2))),
                    (new(2, 3, North), cost(Straight(3))),
                    (new(2, 4, North), cost(Straight(4))),
                    (new(2, 5, North), cost(Straight(5))),
                    (new(2, 6, North), cost(Straight(6))),
                    (new(3, 2, NorthEast), cost(FastRun45)),
                    (new(4, 2, East), cost(FastRun90)),
                    (new(4, 1, SouthEast), cost(FastRun135)),
                    (new(4, 0, South), cost(FastRun180)),
                    (new(1, 2, NorthWest), cost(FastRun45)),
                    (new(0, 2, West), cost(FastRun90)),
                    (new(0, 1, SouthWest), cost(FastRun135)),
                    (new(0, 0, South), cost(FastRun180)),
                ],
            ),
            (
                new(0, 2, East),
                vec![
                    (new(1, 2, East), cost(Straight(1))),
                    (new(2, 2, East), cost(Straight(2))),
                    (new(3, 2, East), cost(Straight(3))),
                    (new(4, 2, East), cost(Straight(4))),
                    (new(5, 2, East), cost(Straight(5))),
                    (new(6, 2, East), cost(Straight(6))),
                    (new(2, 1, SouthEast), cost(FastRun45)),
                    (new(2, 0, South), cost(FastRun90)),
                    (new(1, 0, SouthWest), cost(FastRun135)),
                    (new(0, 0, West), cost(FastRun180)),
                    (new(2, 3, NorthEast), cost(FastRun45)),
                    (new(2, 4, North), cost(FastRun90)),
                    (new(1, 4, NorthWest), cost(FastRun135)),
                    (new(0, 4, West), cost(FastRun180)),
                ],
            ),
            (
                new(2, 1, North),
                vec![
                    (new(2, 2, North), cost(Straight(1))),
                    (new(2, 3, North), cost(Straight(2))),
                    (new(2, 4, North), cost(Straight(3))),
                    (new(2, 5, North), cost(Straight(4))),
                    (new(2, 6, North), cost(Straight(5))),
                    (new(3, 2, East), cost(Search90)),
                    (new(1, 2, West), cost(Search90)),
                    (new(2, 1, South), cost(SpinBack)),
                ],
            ),
            (
                new(3, 2, West),
                vec![
                    (new(2, 2, West), cost(Straight(1))),
                    (new(1, 2, West), cost(Straight(2))),
                    (new(0, 2, West), cost(Straight(3))),
                    (new(2, 3, North), cost(Search90)),
                    (new(2, 1, South), cost(Search90)),
                    (new(3, 2, East), cost(SpinBack)),
                ],
            ),
            (
                new(0, 1, NorthEast),
                vec![
                    (new(1, 2, NorthEast), cost(StraightDiagonal(1))),
                    (new(2, 3, NorthEast), cost(StraightDiagonal(2))),
                    (new(3, 4, NorthEast), cost(StraightDiagonal(3))),
                    (new(4, 5, NorthEast), cost(StraightDiagonal(4))),
                    (new(5, 6, NorthEast), cost(StraightDiagonal(5))),
                    (new(2, 2, East), cost(FastRun45)),
                    (new(2, 1, South), cost(FastRun135)),
                    (new(2, 1, SouthEast), cost(FastRunDiagonal90)),
                ],
            ),
            (
                new(2, 3, SouthWest),
                vec![
                    (new(1, 2, SouthWest), cost(StraightDiagonal(1))),
                    (new(0, 1, SouthWest), cost(StraightDiagonal(2))),
                    (new(0, 2, West), cost(FastRun45)),
                    (new(0, 3, North), cost(FastRun135)),
                    (new(0, 3, NorthWest), cost(FastRunDiagonal90)),
                ],
            ),
            (
                new(1, 0, NorthEast),
                vec![
                    (new(2, 1, NorthEast), cost(StraightDiagonal(1))),
                    (new(3, 2, NorthEast), cost(StraightDiagonal(2))),
                    (new(4, 3, NorthEast), cost(StraightDiagonal(3))),
                    (new(5, 4, NorthEast), cost(StraightDiagonal(4))),
                    (new(6, 5, NorthEast), cost(StraightDiagonal(5))),
                    (new(2, 2, North), cost(FastRun45)),
                    (new(1, 2, West), cost(FastRun135)),
                    (new(1, 2, NorthWest), cost(FastRunDiagonal90)),
                ],
            ),
        ];

        for (src, mut expected) in test_data {
            expected.sort();
            let mut successors = maze.successors(src);
            successors.sort();
            assert_eq!(successors, expected.as_slice());
        }
    }

    #[test]
    fn test_predecessors() {
        use AbsoluteDirection::*;
        use Pattern::*;

        let maze = Maze::<U4, _>::new(cost);

        let new = |x: u16, y: u16, direction: AbsoluteDirection| -> NodeId<U4> {
            NodeId::<U4>::new(x, y, direction)
        };

        let test_data = vec![
            (
                new(2, 2, North),
                vec![
                    (new(2, 1, North), cost(Straight(1))),
                    (new(2, 0, North), cost(Straight(2))),
                    (new(3, 0, NorthWest), cost(FastRun45)),
                    (new(4, 0, West), cost(FastRun90)),
                    (new(4, 1, SouthWest), cost(FastRun135)),
                    (new(4, 2, South), cost(FastRun180)),
                    (new(1, 0, NorthEast), cost(FastRun45)),
                    (new(0, 0, East), cost(FastRun90)),
                    (new(0, 1, SouthEast), cost(FastRun135)),
                    (new(0, 2, South), cost(FastRun180)),
                ],
            ),
            (
                new(3, 2, NorthEast),
                vec![
                    (new(2, 1, NorthEast), cost(Straight(1))),
                    (new(1, 0, NorthEast), cost(Straight(2))),
                    (new(2, 0, North), cost(FastRun45)),
                    (new(3, 0, West), cost(FastRun135)),
                    (new(3, 0, NorthWest), cost(FastRunDiagonal90)),
                ],
            ),
            (
                new(2, 3, NorthEast),
                vec![
                    (new(1, 2, NorthEast), cost(Straight(1))),
                    (new(0, 1, NorthEast), cost(Straight(2))),
                    (new(0, 2, East), cost(FastRun45)),
                    (new(0, 3, South), cost(FastRun135)),
                    (new(0, 3, SouthEast), cost(FastRunDiagonal90)),
                ],
            ),
            (
                new(2, 3, North),
                vec![
                    (new(2, 2, North), cost(Straight(1))),
                    (new(2, 1, North), cost(Straight(2))),
                    (new(2, 0, North), cost(Straight(3))),
                    (new(3, 2, West), cost(Search90)),
                    (new(1, 2, East), cost(Search90)),
                    (new(2, 3, South), cost(SpinBack)),
                ],
            ),
        ];

        for (src, mut expected) in test_data {
            expected.sort();
            let mut predecessors = maze.predecessors(src);
            predecessors.sort();
            assert_eq!(predecessors, expected.as_slice());
        }
    }
}
