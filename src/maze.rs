mod direction;
mod node;

use core::fmt::Debug;
use core::marker::PhantomData;
use core::ops::{Add, Mul};

use generic_array::{ArrayLength, GenericArray};
use heapless::{consts::*, Vec};
use typenum::{PowerOfTwo, Unsigned};

use crate::operator::{Graph, GraphConverter};
use crate::pattern::Pattern;
use direction::{AbsoluteDirection, RelativeDirection};
use node::{Location, Node, NodeId, Position, SearchNodeId};

#[derive(Clone)]
struct WallPosition<N> {
    x: u16,
    y: u16,
    z: bool, //false: up, true: right
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
    Node<N>: Debug,
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

    fn is_checked_by_position(&self, position: Position<N>) -> bool {
        if let Some(wall_position) = WallPosition::from_position(position) {
            self.is_checked(wall_position)
        } else {
            true
        }
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
    Node<N>: Debug,
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
            .push((relative_node(0, 2, Front), self.cost(Straight(2))))
            .unwrap();
        for i in 1..N::I16 {
            if is_wall_relative(0, 2 * i + 1) {
                break;
            }
            succs
                .push((
                    relative_node(0, 2 * i + 2, Front),
                    self.cost(Straight((2 * i + 2) as u16)),
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
            .push((relative_node(2, -1, BackRight), self.cost(FastRun135)))
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
            .push((relative_node(-1, 2, BackLeft), self.cost(FastRun135)))
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
    Node<N>: Debug,
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
                _ => unreachable!(),
            },
            VerticalBound => match node.direction() {
                NorthEast | SouthEast | SouthWest | NorthWest => {
                    self.vertical_bound_diagonal_neighbors(node, is_successors)
                }
                _ => unreachable!(),
            },
            HorizontalBound => match node.direction() {
                NorthEast | SouthEast | SouthWest | NorthWest => {
                    self.horizontal_bound_diagonal_neighbors(node, is_successors)
                }
                _ => unreachable!(),
            },
        };
        node_neighbors
            .into_iter()
            .filter_map(|(node, cost)| node.to_node_id().map(|node_id| (node_id, cost)))
            .collect()
    }
}

impl<N, F> Maze<N, F>
where
    N: Mul<N> + Mul<U2> + Unsigned + PowerOfTwo,
    <N as Mul<U2>>::Output: Add<U10>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(Node<N>, u16)>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(SearchNodeId<N>, u16)>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
    F: Fn(Pattern) -> u16,
    Node<N>: Debug,
{
    fn search_node_neighbors(
        &self,
        node_id: SearchNodeId<N>,
        is_successors: bool,
    ) -> <Self as Graph<SearchNodeId<N>, u16>>::Edges {
        use AbsoluteDirection::*;
        use Pattern::*;
        use RelativeDirection::*;

        let node = node_id.as_node();

        let is_wall_relative = self.is_wall_relative_fn(&node, North, is_successors);
        let relative_node = Self::relative_node_fn(&node, North, is_successors);

        let mut neighbors = Vec::<(Node<N>, u16), U4>::new();
        neighbors
            .push((relative_node(0, 0, Back), self.cost(SpinBack)))
            .unwrap();
        if !is_wall_relative(1, 1) {
            neighbors
                .push((relative_node(1, 1, Right), self.cost(Search90)))
                .unwrap();
        }
        if !is_wall_relative(-1, 1) {
            neighbors
                .push((relative_node(-1, 1, Left), self.cost(Search90)))
                .unwrap();
        }
        if !is_wall_relative(0, 2) {
            neighbors
                .push((relative_node(0, 2, Front), self.cost(Straight(2))))
                .unwrap();
        }
        neighbors
            .into_iter()
            .filter_map(|(node, cost)| node.to_search_node_id().map(|node_id| (node_id, cost)))
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
    Node<N>: Debug,
{
    type Edges = Vec<(NodeId<N>, u16), <<N as Mul<U2>>::Output as Add<U10>>::Output>;

    fn successors(&self, node_id: NodeId<N>) -> Self::Edges {
        self.neighbors(node_id, true)
    }

    fn predecessors(&self, node_id: NodeId<N>) -> Self::Edges {
        self.neighbors(node_id, false)
    }
}

impl<N, F> Graph<SearchNodeId<N>, u16> for Maze<N, F>
where
    N: Mul<N> + Mul<U2> + Unsigned + PowerOfTwo,
    <N as Mul<U2>>::Output: Add<U10>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(Node<N>, u16)>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(SearchNodeId<N>, u16)>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
    F: Fn(Pattern) -> u16,
    Node<N>: Debug,
{
    type Edges = Vec<(SearchNodeId<N>, u16), <<N as Mul<U2>>::Output as Add<U10>>::Output>;

    fn successors(&self, node: SearchNodeId<N>) -> Self::Edges {
        self.search_node_neighbors(node, true)
    }

    fn predecessors(&self, node: SearchNodeId<N>) -> Self::Edges {
        self.search_node_neighbors(node, false)
    }
}

impl<N, F> Maze<N, F>
where
    N: Mul<N> + Unsigned + PowerOfTwo + Debug,
    <N as Mul<N>>::Output: Mul<U2> + Mul<U4>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
    <<N as Mul<N>>::Output as Mul<U4>>::Output: ArrayLength<SearchNodeId<N>>,
    <<N as Mul<N>>::Output as Mul<U4>>::Output: ArrayLength<Position<N>>,
    F: Fn(Pattern) -> u16,
{
    fn relative_wall_position_fn<'a>(
        node: &'a Node<N>,
        base_dir: AbsoluteDirection,
    ) -> impl Fn(i16, i16) -> Position<N> + 'a {
        move |x: i16, y: i16| -> Position<N> { node.relative_position(x, y, base_dir).unwrap() }
    }

    fn wall_positions_on_passage_from_cell(
        src: &Node<N>,
        dst: &Node<N>,
    ) -> Vec<Position<N>, <<N as Mul<N>>::Output as Mul<U4>>::Output> {
        use AbsoluteDirection::*;
        use RelativeDirection::*;

        let relative_wall_position = Self::relative_wall_position_fn(&src, North);

        let mut walls = Vec::new();
        walls.push(relative_wall_position(0, 1)).unwrap();
        match src.difference(dst, North) {
            (1, 2, FrontRight) | (2, 2, Right) => {
                walls.push(relative_wall_position(1, 2)).unwrap();
            }
            (2, 1, BackRight) | (2, 0, Back) => {
                walls.push(relative_wall_position(1, 2)).unwrap();
                walls.push(relative_wall_position(2, 1)).unwrap();
            }
            (-1, 2, FrontLeft) | (-2, 2, Left) => {
                walls.push(relative_wall_position(-1, 2)).unwrap();
            }
            (-2, 1, BackLeft) | (-2, 0, Back) => {
                walls.push(relative_wall_position(-1, 2)).unwrap();
                walls.push(relative_wall_position(-2, 1)).unwrap();
            }
            (0, y, Front) => {
                for i in (3..y + 1).step_by(2) {
                    walls.push(relative_wall_position(0, i)).unwrap();
                }
            }
            _ => unreachable!(),
        }
        walls
    }

    fn wall_positions_on_passage_from_bound_straight(
        src: &Node<N>,
        dst: &Node<N>,
    ) -> Vec<Position<N>, <<N as Mul<N>>::Output as Mul<U4>>::Output> {
        use AbsoluteDirection::*;
        use RelativeDirection::*;

        let relative_wall_position = Self::relative_wall_position_fn(&src, North);

        let mut walls = Vec::new();
        match src.difference(dst, North) {
            (1, 1, Right) => {
                walls.push(relative_wall_position(1, 1)).unwrap();
            }
            (-1, 1, Left) => {
                walls.push(relative_wall_position(-1, 1)).unwrap();
            }
            (0, y, Front) => {
                for i in (2..y + 1).step_by(2) {
                    walls.push(relative_wall_position(0, i)).unwrap();
                }
            }
            _ => unreachable!(),
        }
        walls
    }

    fn wall_positions_on_passage_from_vertical_bound_diagonal(
        src: &Node<N>,
        dst: &Node<N>,
    ) -> Vec<Position<N>, <<N as Mul<N>>::Output as Mul<U4>>::Output> {
        use AbsoluteDirection::*;
        use RelativeDirection::*;

        let relative_wall_position = Self::relative_wall_position_fn(&src, NorthEast);

        let mut walls = Vec::new();
        walls.push(relative_wall_position(1, 1)).unwrap();
        match src.difference(dst, NorthEast) {
            (1, 2, FrontLeft) => (),
            (0, 2, Left) | (0, 2, BackLeft) => {
                walls.push(relative_wall_position(0, 2)).unwrap();
            }
            (x, y, Front) if x == y => {
                for i in 2..x + 1 {
                    walls.push(relative_wall_position(i, i)).unwrap();
                }
            }
            _ => unreachable!(),
        }
        walls
    }

    fn wall_positions_on_passage_from_horizontal_bound_diagonal(
        src: &Node<N>,
        dst: &Node<N>,
    ) -> Vec<Position<N>, <<N as Mul<N>>::Output as Mul<U4>>::Output> {
        use AbsoluteDirection::*;
        use RelativeDirection::*;

        let relative_wall_position = Self::relative_wall_position_fn(&src, NorthEast);

        let mut walls = Vec::new();
        walls.push(relative_wall_position(1, 1)).unwrap();
        match src.difference(dst, NorthEast) {
            (2, 1, FrontRight) => (),
            (2, 0, Right) | (2, 0, BackRight) => {
                walls.push(relative_wall_position(2, 0)).unwrap();
            }
            (x, y, Front) if x == y => {
                for i in 2..x + 1 {
                    walls.push(relative_wall_position(i, i)).unwrap();
                }
            }
            _ => unreachable!(),
        }
        walls
    }
}

impl<N, F> GraphConverter<NodeId<N>, SearchNodeId<N>> for Maze<N, F>
where
    N: Mul<N> + Unsigned + PowerOfTwo + Debug,
    <N as Mul<N>>::Output: Mul<U2> + Mul<U4>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
    <<N as Mul<N>>::Output as Mul<U4>>::Output: ArrayLength<SearchNodeId<N>>,
    <<N as Mul<N>>::Output as Mul<U4>>::Output: ArrayLength<Node<N>>,
    <<N as Mul<N>>::Output as Mul<U4>>::Output: ArrayLength<Position<N>>,
    F: Fn(Pattern) -> u16,
{
    type SearchNodes = Vec<SearchNodeId<N>, <<N as Mul<N>>::Output as Mul<U4>>::Output>;

    //NOTE: Suppose that one of the cells nearest to start and goal is not need to be visited.
    //The cells should be directly connected to the terminals of the given path.
    fn convert_to_checker_nodes<Nodes: IntoIterator<Item = NodeId<N>>>(
        &self,
        path: Nodes,
    ) -> Self::SearchNodes {
        use AbsoluteDirection::*;
        use Location::*;

        let mut src: Option<Node<N>> = None;
        let mut positions = Vec::<Position<N>, <<N as Mul<N>>::Output as Mul<U4>>::Output>::new();
        for dst in path.into_iter() {
            let dst = dst.as_node();
            if let Some(src) = src {
                let positions_on_passage = match src.location() {
                    Cell => match src.direction() {
                        North | East | South | West => {
                            Self::wall_positions_on_passage_from_cell(&src, &dst)
                        }
                        _ => unreachable!(),
                    },
                    VerticalBound => match src.direction() {
                        East | West => {
                            Self::wall_positions_on_passage_from_bound_straight(&src, &dst)
                        }
                        NorthEast | SouthEast | SouthWest | NorthWest => {
                            Self::wall_positions_on_passage_from_vertical_bound_diagonal(&src, &dst)
                        }
                        _ => unreachable!(),
                    },
                    HorizontalBound => match src.direction() {
                        North | South => {
                            Self::wall_positions_on_passage_from_bound_straight(&src, &dst)
                        }
                        NorthEast | SouthEast | SouthWest | NorthWest => {
                            Self::wall_positions_on_passage_from_horizontal_bound_diagonal(
                                &src, &dst,
                            )
                        }
                        _ => unreachable!(),
                    },
                };
                positions.extend_from_slice(&positions_on_passage).unwrap();
            }
            src = Some(dst);
        }

        let mut checker_nodes = Vec::<Node<N>, <<N as Mul<N>>::Output as Mul<U4>>::Output>::new();
        for (&src, &dst) in positions.iter().zip(positions.iter().skip(1)) {
            if self.is_checked_by_position(dst) {
                continue;
            }
            let mut add_if_no_wall = |dx, dy, direction| {
                if !self.is_wall_by_position(dst.relative_position(dx, dy)) {
                    checker_nodes
                        .push(dst.relative_node(dx, dy, direction))
                        .unwrap();
                }
            };
            match dst.location() {
                VerticalBound => match dst.difference(&src) {
                    (-1, -1) => {
                        add_if_no_wall(-2, 0, East);
                        add_if_no_wall(-1, 1, South);
                    }
                    (-2, 0) => {
                        add_if_no_wall(-1, -1, North);
                        add_if_no_wall(-1, 1, South);
                    }
                    (-1, 1) => {
                        add_if_no_wall(-1, -1, North);
                        add_if_no_wall(-2, 0, East);
                    }
                    (1, 1) => {
                        add_if_no_wall(2, 0, West);
                        add_if_no_wall(1, -1, North);
                    }
                    (2, 0) => {
                        add_if_no_wall(1, 1, South);
                        add_if_no_wall(1, -1, North);
                    }
                    (1, -1) => {
                        add_if_no_wall(2, 0, West);
                        add_if_no_wall(1, 1, South);
                    }
                    _ => unreachable!(),
                },
                HorizontalBound => match dst.difference(&src) {
                    (-1, -1) => {
                        add_if_no_wall(0, -2, North);
                        add_if_no_wall(1, -1, West);
                    }
                    (0, -2) => {
                        add_if_no_wall(-1, -1, East);
                        add_if_no_wall(1, -1, West);
                    }
                    (1, -1) => {
                        add_if_no_wall(-1, -1, East);
                        add_if_no_wall(0, -2, North);
                    }
                    (-1, 1) => {
                        add_if_no_wall(0, 2, South);
                        add_if_no_wall(1, 1, West);
                    }
                    (0, 2) => {
                        add_if_no_wall(-1, 1, East);
                        add_if_no_wall(1, 1, West);
                    }
                    (1, 1) => {
                        add_if_no_wall(-1, 1, East);
                        add_if_no_wall(0, 2, South);
                    }
                    _ => unreachable!(),
                },
                Cell => unreachable!(),
            }
        }
        checker_nodes
            .into_iter()
            .filter_map(|node| node.to_search_node_id())
            .collect()
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
    fn test_node_successors() {
        use AbsoluteDirection::*;
        use Pattern::*;

        let maze = Maze::<U4, _>::new(cost);

        let new = |x: u16, y: u16, direction: AbsoluteDirection| -> NodeId<U4> {
            NodeId::<U4>::new(x, y, direction).unwrap()
        };

        let test_data = vec![
            (
                new(2, 0, North),
                vec![
                    (new(2, 2, North), cost(Straight(2))),
                    (new(2, 4, North), cost(Straight(4))),
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
                    (new(2, 2, East), cost(Straight(2))),
                    (new(4, 2, East), cost(Straight(4))),
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
                new(0, 1, NorthEast),
                vec![
                    (new(1, 2, NorthEast), cost(StraightDiagonal(1))),
                    (new(2, 3, NorthEast), cost(StraightDiagonal(2))),
                    (new(3, 4, NorthEast), cost(StraightDiagonal(3))),
                    (new(4, 5, NorthEast), cost(StraightDiagonal(4))),
                    (new(5, 6, NorthEast), cost(StraightDiagonal(5))),
                    (new(2, 2, East), cost(FastRun45)),
                    (new(2, 0, South), cost(FastRun135)),
                    (new(2, 1, SouthEast), cost(FastRunDiagonal90)),
                ],
            ),
            (
                new(2, 3, SouthWest),
                vec![
                    (new(1, 2, SouthWest), cost(StraightDiagonal(1))),
                    (new(0, 1, SouthWest), cost(StraightDiagonal(2))),
                    (new(0, 2, West), cost(FastRun45)),
                    (new(0, 4, North), cost(FastRun135)),
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
                    (new(0, 2, West), cost(FastRun135)),
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
    fn test_node_predecessors() {
        use AbsoluteDirection::*;
        use Pattern::*;

        let maze = Maze::<U4, _>::new(cost);

        let new = |x: u16, y: u16, direction: AbsoluteDirection| -> NodeId<U4> {
            NodeId::<U4>::new(x, y, direction).unwrap()
        };

        let test_data = vec![
            (
                new(2, 2, North),
                vec![
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
                    (new(4, 0, West), cost(FastRun135)),
                    (new(3, 0, NorthWest), cost(FastRunDiagonal90)),
                ],
            ),
            (
                new(2, 3, NorthEast),
                vec![
                    (new(1, 2, NorthEast), cost(Straight(1))),
                    (new(0, 1, NorthEast), cost(Straight(2))),
                    (new(0, 2, East), cost(FastRun45)),
                    (new(0, 4, South), cost(FastRun135)),
                    (new(0, 3, SouthEast), cost(FastRunDiagonal90)),
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

    #[test]
    fn test_search_node_successors() {
        use AbsoluteDirection::*;
        use Pattern::*;

        let maze = Maze::<U4, _>::new(cost);

        let new = |x, y, direction| SearchNodeId::<U4>::new(x, y, direction).unwrap();

        let test_data = vec![
            (
                new(2, 1, North),
                vec![
                    (new(2, 1, South), cost(SpinBack)),
                    (new(3, 2, East), cost(Search90)),
                    (new(1, 2, West), cost(Search90)),
                    (new(2, 3, North), cost(Straight(2))),
                ],
            ),
            (
                new(1, 2, East),
                vec![
                    (new(1, 2, West), cost(SpinBack)),
                    (new(2, 1, South), cost(Search90)),
                    (new(2, 3, North), cost(Search90)),
                    (new(3, 2, East), cost(Straight(2))),
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
    fn test_seach_node_predecessors() {
        use AbsoluteDirection::*;
        use Pattern::*;

        let maze = Maze::<U4, _>::new(cost);

        let new = |x, y, direction| SearchNodeId::<U4>::new(x, y, direction).unwrap();

        let test_data = vec![
            (
                new(2, 3, North),
                vec![
                    (new(2, 3, South), cost(SpinBack)),
                    (new(3, 2, West), cost(Search90)),
                    (new(1, 2, East), cost(Search90)),
                    (new(2, 1, North), cost(Straight(2))),
                ],
            ),
            (
                new(3, 2, East),
                vec![
                    (new(3, 2, West), cost(SpinBack)),
                    (new(2, 1, North), cost(Search90)),
                    (new(2, 3, South), cost(Search90)),
                    (new(1, 2, East), cost(Straight(2))),
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

    #[test]
    fn test_convert_to_checker_nodes() {
        use AbsoluteDirection::*;

        let new = |x, y, direction| NodeId::<U4>::new(x, y, direction).unwrap();
        let new_search = |x, y, direction| SearchNodeId::<U4>::new(x, y, direction).unwrap();
        let new_wall = |x, y, z| WallPosition::new(x, y, z);

        let test_data = vec![
            (
                vec![
                    new_wall(0, 0, true),
                    new_wall(0, 1, true),
                    new_wall(1, 0, false),
                    new_wall(1, 1, false),
                    new_wall(1, 2, true),
                    new_wall(2, 2, false),
                ],
                vec![
                    new(0, 0, North),
                    new(0, 2, North),
                    new(1, 4, NorthEast),
                    new(2, 5, NorthEast),
                    new(4, 6, East),
                    new(5, 4, SouthWest),
                    new(4, 2, South),
                    new(2, 0, West),
                ],
                vec![
                    new_search(0, 5, South),
                    new_search(1, 6, East),
                    new_search(6, 3, North),
                    new_search(5, 2, West),
                    new_search(5, 0, West),
                    new_search(3, 2, East),
                ],
            ),
            (
                vec![new_wall(1, 2, true)],
                vec![
                    new(0, 0, North),
                    new(1, 2, NorthEast),
                    new(2, 4, North),
                    new(4, 4, South),
                ],
                vec![
                    new_search(0, 3, South),
                    new_search(2, 1, North),
                    new_search(3, 2, West),
                    new_search(1, 4, East),
                    new_search(1, 6, East),
                    new_search(5, 6, West),
                ],
            ),
        ];

        for (walls, path, mut expected) in test_data {
            let mut maze = Maze::<U4, _>::new(cost);
            for wall in walls {
                maze.check_wall(wall, true);
            }
            expected.sort();
            let mut checker_nodes = maze.convert_to_checker_nodes(path);
            checker_nodes.sort();
            assert_eq!(checker_nodes, expected.as_slice());
        }
    }
}
