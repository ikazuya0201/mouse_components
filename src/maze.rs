mod direction;
mod node;
mod pose_converter;
mod wall;

use core::cell::RefCell;
use core::convert::{TryFrom, TryInto};
use core::fmt::Debug;
use core::marker::PhantomData;
use core::ops::{Add, Mul};

use generic_array::GenericArray;
use heapless::{consts::*, Vec};
use typenum::{PowerOfTwo, Unsigned};

use crate::data_types::{Pose, SearchKind};
use crate::obstacle_detector::Obstacle;
use crate::solver::{
    CannotCheckError, Converter, Graph, GraphConverter, NodeChecker, ObstacleInterpreter,
};
use crate::traits::Math;
use crate::utils::{array_length::ArrayLength, forced_vec::ForcedVec, itertools::repeat_n};
pub use direction::{AbsoluteDirection, RelativeDirection};
pub use node::Position;
use node::{Location, Node};
pub use node::{NodeId, SearchNodeId};
pub use pose_converter::ConversionError;
use pose_converter::PoseConverter;
use uom::si::{f32::Length, ratio::ratio};
pub use wall::{WallDirection, WallPosition};

#[derive(Clone, Copy, Debug)]
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

pub struct Maze<N, M>
where
    N: Mul<N> + Unsigned + PowerOfTwo,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
{
    pub(crate) wall_existence_probs:
        RefCell<GenericArray<f32, <<N as Mul<N>>::Output as Mul<U2>>::Output>>,
    wall_prob_threshold: f32,
    costs: fn(Pattern) -> u16,
    converter: PoseConverter<M>,
}

impl<N, M> Maze<N, M>
where
    N: Mul<N> + Unsigned + PowerOfTwo,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
{
    fn new(
        wall_prob_threshold: f32,
        costs: fn(Pattern) -> u16,
        square_width: Length,
        wall_width: Length,
        ignore_radius_from_pillar: Length,
    ) -> Self {
        let probs = repeat_n(0.5, <<N as Mul<N>>::Output as Mul<U2>>::Output::USIZE)
            .collect::<GenericArray<_, <<N as Mul<N>>::Output as Mul<U2>>::Output>>();
        Maze {
            wall_prob_threshold,
            wall_existence_probs: RefCell::new(probs),
            costs,
            converter: PoseConverter::new(square_width, wall_width, ignore_radius_from_pillar),
        }
    }

    fn initialize_with_str(&self, input: &str) {
        input.lines().enumerate().for_each(|(y, line)| {
            line.chars().enumerate().for_each(|(x, c)| {
                if y % 2 == 0 {
                    if x % 4 != 1 {
                        return;
                    }
                    let x = x / 4;
                    let position = Position::<N>::new(2 * x as i16, 2 * N::I16 - y as i16 - 1);
                    if c == '-' {
                        self.check_wall_by_position(position, true)
                    } else {
                        self.check_wall_by_position(position, false)
                    }
                } else {
                    if x % 4 != 0 {
                        return;
                    }
                    let x = x / 4;
                    let position = Position::<N>::new(2 * x as i16 - 1, 2 * N::I16 - y as i16 - 1);
                    if c == '|' {
                        self.check_wall_by_position(position, true);
                    } else {
                        self.check_wall_by_position(position, false);
                    }
                }
            });
        });
    }

    fn initialize(&self) {
        for i in 0..WallPosition::<N>::max() + 1 {
            self.check_wall(
                WallPosition::new(i, WallPosition::<N>::max(), WallDirection::Up)
                    .unwrap_or_else(|err| unreachable!("{:?}", err)),
                true,
            );
            self.check_wall(
                WallPosition::new(WallPosition::<N>::max(), i, WallDirection::Right)
                    .unwrap_or_else(|err| unreachable!("{:?}", err)),
                true,
            );
        }
        self.check_wall(
            WallPosition::new(0, 0, WallDirection::Right)
                .unwrap_or_else(|err| unreachable!("{:?}", err)),
            true,
        );
        self.check_wall(
            WallPosition::new(0, 0, WallDirection::Up)
                .unwrap_or_else(|err| unreachable!("{:?}", err)),
            false,
        );
    }

    pub fn check_wall(&self, position: WallPosition<N>, is_wall: bool) {
        self.check_wall_by_index(position.as_index(), is_wall);
    }

    fn check_wall_by_position(&self, position: Position<N>, is_wall: bool) {
        if let Ok(wall_position) = WallPosition::try_from(position) {
            self.check_wall(wall_position, is_wall);
        }
    }

    #[cfg(test)]
    fn wall_existence_probs<'a>(
        &'a self,
    ) -> &'a RefCell<GenericArray<f32, <<N as Mul<N>>::Output as Mul<U2>>::Output>> {
        &self.wall_existence_probs
    }

    #[inline]
    fn check_wall_by_index(&self, index: usize, is_wall: bool) {
        self.update_wall_by_index(index, is_wall);
    }

    #[inline]
    fn update_wall_by_index(&self, index: usize, is_wall: bool) {
        self.wall_existence_probs.borrow_mut()[index] = if is_wall { 1.0 } else { 0.0 };
    }

    #[inline]
    fn is_wall(&self, position: WallPosition<N>) -> bool {
        let prob = self.wall_existence_probs.borrow()[position.as_index()];
        return prob > 1.0 - self.wall_prob_threshold;
    }

    pub fn is_wall_by_position(&self, position: Position<N>) -> bool {
        if let Ok(wall_position) = WallPosition::try_from(position) {
            self.is_wall(wall_position)
        } else {
            true
        }
    }

    #[inline]
    fn is_checked(&self, position: WallPosition<N>) -> bool {
        let prob = self.wall_existence_probs.borrow()[position.as_index()];
        return prob < self.wall_prob_threshold || prob > 1.0 - self.wall_prob_threshold;
    }

    fn is_checked_by_position(&self, position: Position<N>) -> bool {
        if let Ok(wall_position) = WallPosition::try_from(position) {
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
    ) -> impl Fn(i16, i16) -> bool + 'a
    where
        N: core::fmt::Debug,
    {
        move |x: i16, y: i16| -> bool {
            if is_successors {
                self.is_wall_by_position(
                    node.get_relative_position(x, y, base_dir)
                        .unwrap_or_else(|err| unreachable!("This is a bug: {:?}", err)),
                )
            } else {
                self.is_wall_by_position(
                    node.get_relative_position(-x, -y, base_dir)
                        .unwrap_or_else(|err| unreachable!("This is a bug: {:?}", err)),
                )
            }
        }
    }

    fn relative_node_fn<'a>(
        node: &'a Node<N>,
        base_dir: AbsoluteDirection,
        is_successors: bool,
    ) -> impl Fn(i16, i16, RelativeDirection) -> Node<N> + 'a
    where
        N: core::fmt::Debug,
    {
        move |x: i16, y: i16, direction: RelativeDirection| -> Node<N> {
            if is_successors {
                node.get_relative_node(x, y, direction, base_dir)
                    .unwrap_or_else(|err| unreachable!("This is a bug: {:?}", err))
            } else {
                node.get_relative_node(-x, -y, direction, base_dir)
                    .unwrap_or_else(|err| unreachable!("This is a bug: {:?}", err))
            }
        }
    }
}

impl<N, M> Maze<N, M>
where
    N: Mul<N> + Mul<U2> + Unsigned + PowerOfTwo + Debug,
    <N as Mul<U2>>::Output: Add<U10>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(Node<N>, u16)>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
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
            || -> ForcedVec<(Node<N>, u16), <<N as Mul<U2>>::Output as Add<U10>>::Output> {
                let mut succs = ForcedVec::new();
                if is_wall_relative(1, 2) {
                    return succs;
                }
                succs.push((relative_node(1, 2, FrontRight), self.cost(FastRun45)));
                succs.push((relative_node(2, 2, Right), self.cost(FastRun90)));

                if is_wall_relative(2, 1) {
                    return succs;
                }
                succs.push((relative_node(2, 1, BackRight), self.cost(FastRun135)));
                succs.push((relative_node(2, 0, Back), self.cost(FastRun180)));
                succs
            };

        let left_successors = || -> ForcedVec<(Node<N>, u16), U4> {
            let mut succs = ForcedVec::new();
            if is_wall_relative(-1, 2) {
                return succs;
            }
            succs.push((relative_node(-1, 2, FrontLeft), self.cost(FastRun45)));
            succs.push((relative_node(-2, 2, Left), self.cost(FastRun90)));

            if is_wall_relative(-2, 1) {
                return succs;
            }
            succs.push((relative_node(-2, 1, BackLeft), self.cost(FastRun135)));
            succs.push((relative_node(-2, 0, Back), self.cost(FastRun180)));
            succs
        };

        let mut succs = right_successors();
        succs.extend_from_slice(&left_successors());

        succs.push((relative_node(0, 2, Front), self.cost(Straight(1))));
        for i in 1..N::I16 {
            if is_wall_relative(0, 2 * i + 1) {
                break;
            }
            succs.push((
                relative_node(0, 2 * i + 2, Front),
                self.cost(Straight((i + 1) as u16)),
            ));
        }

        succs.into()
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

        let mut succs = ForcedVec::new();
        if is_wall_relative(1, 1) {
            return succs.into();
        }
        succs.push((relative_node(1, 1, Front), self.cost(StraightDiagonal(1))));
        succs.push((relative_node(2, 1, FrontRight), self.cost(FastRun45)));

        if is_wall_relative(2, 0) {
            return succs.into();
        }
        succs.push((relative_node(2, -1, BackRight), self.cost(FastRun135)));
        succs.push((relative_node(2, 0, Right), self.cost(FastRunDiagonal90)));

        for i in 2..N::I16 * 2 {
            if is_wall_relative(i, i) {
                break;
            }
            succs.push((
                relative_node(i, i, Front),
                self.cost(StraightDiagonal(i as u16)),
            ));
        }

        succs.into()
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

        let mut succs = ForcedVec::new();
        if is_wall_relative(1, 1) {
            return succs.into();
        }
        succs.push((relative_node(1, 1, Front), self.cost(StraightDiagonal(1))));
        succs.push((relative_node(1, 2, FrontLeft), self.cost(FastRun45)));

        if is_wall_relative(0, 2) {
            return succs.into();
        }
        succs.push((relative_node(-1, 2, BackLeft), self.cost(FastRun135)));
        succs.push((relative_node(0, 2, Left), self.cost(FastRunDiagonal90)));

        for i in 2..N::I16 * 2 {
            if is_wall_relative(i, i) {
                break;
            }
            succs.push((
                relative_node(i, i, Front),
                self.cost(StraightDiagonal(i as u16)),
            ));
        }

        succs.into()
    }
}

impl<N, M> Maze<N, M>
where
    N: Mul<N> + Mul<U2> + Unsigned + PowerOfTwo + Debug,
    <N as Mul<U2>>::Output: Add<U10>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(Node<N>, u16)>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(NodeId<N>, u16)>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
    Node<N>: Debug,
{
    fn neighbors(
        &self,
        node_id: NodeId<N>,
        is_successors: bool,
    ) -> <Self as Graph<NodeId<N>>>::Edges {
        use AbsoluteDirection::*;
        use Location::*;

        let node = node_id.to_node();
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
            .filter_map(|(node, cost)| node.try_into().map(|node_id| (node_id, cost)).ok())
            .collect()
    }
}

impl<N, M> Maze<N, M>
where
    N: Mul<N> + Unsigned + PowerOfTwo + Debug,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
    Node<N>: Debug,
{
    fn search_node_neighbors(
        &self,
        node_id: SearchNodeId<N>,
        is_successors: bool,
    ) -> <Self as Graph<SearchNodeId<N>>>::Edges {
        use AbsoluteDirection::*;
        use Pattern::*;
        use RelativeDirection::*;

        let node = node_id.to_node();

        let is_wall_relative = self.is_wall_relative_fn(&node, North, is_successors);
        let relative_node = Self::relative_node_fn(&node, North, is_successors);

        let mut neighbors = ForcedVec::<(Node<N>, u16), U4>::new();
        neighbors.push((relative_node(0, 0, Back), self.cost(SpinBack)));
        if !is_wall_relative(1, 1) {
            neighbors.push((relative_node(1, 1, Right), self.cost(Search90)));
        }
        if !is_wall_relative(-1, 1) {
            neighbors.push((relative_node(-1, 1, Left), self.cost(Search90)));
        }
        if !is_wall_relative(0, 2) {
            neighbors.push((relative_node(0, 2, Front), self.cost(Straight(1))));
        }
        <ForcedVec<(Node<N>, u16), U4> as Into<Vec<(Node<N>, u16), U4>>>::into(neighbors)
            .into_iter()
            .filter_map(|(node, cost)| node.try_into().map(|node_id| (node_id, cost)).ok())
            .collect()
    }
}

impl<N, M> Graph<NodeId<N>> for Maze<N, M>
where
    N: Mul<N> + Mul<U2> + Unsigned + PowerOfTwo + Debug,
    <N as Mul<U2>>::Output: Add<U10>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(Node<N>, u16)>,
    <<N as Mul<U2>>::Output as Add<U10>>::Output: ArrayLength<(NodeId<N>, u16)>,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
{
    type Cost = u16;
    type Edges = Vec<(NodeId<N>, u16), <<N as Mul<U2>>::Output as Add<U10>>::Output>;

    fn successors(&self, node_id: &NodeId<N>) -> Self::Edges {
        self.neighbors(*node_id, true)
    }

    fn predecessors(&self, node_id: &NodeId<N>) -> Self::Edges {
        self.neighbors(*node_id, false)
    }
}

impl<N, M> Graph<SearchNodeId<N>> for Maze<N, M>
where
    N: Mul<N> + Unsigned + PowerOfTwo + Debug,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
    Node<N>: Debug,
{
    type Cost = u16;
    type Edges = Vec<(SearchNodeId<N>, u16), U4>;

    fn successors(&self, node: &SearchNodeId<N>) -> Self::Edges {
        self.search_node_neighbors(*node, true)
    }

    fn predecessors(&self, node: &SearchNodeId<N>) -> Self::Edges {
        self.search_node_neighbors(*node, false)
    }
}

impl<N, M> Maze<N, M>
where
    N: Mul<N> + Unsigned + PowerOfTwo + Debug,
    <N as Mul<N>>::Output: Mul<U2> + Mul<U4>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
    <<N as Mul<N>>::Output as Mul<U4>>::Output: ArrayLength<SearchNodeId<N>>,
    <<N as Mul<N>>::Output as Mul<U4>>::Output: ArrayLength<Position<N>>,
{
    fn relative_wall_position_fn<'a>(
        node: &'a Node<N>,
        base_dir: AbsoluteDirection,
    ) -> impl Fn(i16, i16) -> Position<N> + 'a {
        move |x: i16, y: i16| -> Position<N> {
            node.get_relative_position(x, y, base_dir)
                .unwrap_or_else(|_| unreachable!())
        }
    }

    fn wall_positions_on_passage_from_cell(
        src: &Node<N>,
        dst: &Node<N>,
    ) -> ForcedVec<Position<N>, <<N as Mul<N>>::Output as Mul<U4>>::Output> {
        use AbsoluteDirection::*;
        use RelativeDirection::*;

        let relative_wall_position = Self::relative_wall_position_fn(&src, North);

        let mut walls = ForcedVec::new();
        walls.push(relative_wall_position(0, 1));
        match src.difference(dst, North) {
            (1, 2, FrontRight) | (2, 2, Right) => {
                walls.push(relative_wall_position(1, 2));
            }
            (2, 1, BackRight) | (2, 0, Back) => {
                walls.push(relative_wall_position(1, 2));
                walls.push(relative_wall_position(2, 1));
            }
            (-1, 2, FrontLeft) | (-2, 2, Left) => {
                walls.push(relative_wall_position(-1, 2));
            }
            (-2, 1, BackLeft) | (-2, 0, Back) => {
                walls.push(relative_wall_position(-1, 2));
                walls.push(relative_wall_position(-2, 1));
            }
            (0, y, Front) => {
                for i in (3..y + 1).step_by(2) {
                    walls.push(relative_wall_position(0, i));
                }
            }
            _ => unreachable!(),
        }
        walls
    }

    fn wall_positions_on_passage_from_bound_straight(
        src: &Node<N>,
        dst: &Node<N>,
    ) -> ForcedVec<Position<N>, <<N as Mul<N>>::Output as Mul<U4>>::Output> {
        use AbsoluteDirection::*;
        use RelativeDirection::*;

        let relative_wall_position = Self::relative_wall_position_fn(&src, North);

        let mut walls = ForcedVec::new();
        match src.difference(dst, North) {
            (1, 1, Right) => {
                walls.push(relative_wall_position(1, 1));
            }
            (-1, 1, Left) => {
                walls.push(relative_wall_position(-1, 1));
            }
            (0, y, Front) => {
                for i in (2..y + 1).step_by(2) {
                    walls.push(relative_wall_position(0, i));
                }
            }
            _ => unreachable!(),
        }
        walls
    }

    fn wall_positions_on_passage_from_vertical_bound_diagonal(
        src: &Node<N>,
        dst: &Node<N>,
    ) -> ForcedVec<Position<N>, <<N as Mul<N>>::Output as Mul<U4>>::Output> {
        use AbsoluteDirection::*;
        use RelativeDirection::*;

        let relative_wall_position = Self::relative_wall_position_fn(&src, NorthEast);

        let mut walls = ForcedVec::new();
        walls.push(relative_wall_position(1, 1));
        match src.difference(dst, NorthEast) {
            (1, 2, FrontLeft) => (),
            (0, 2, Left) | (-1, 2, BackLeft) => {
                walls.push(relative_wall_position(0, 2));
            }
            (x, y, Front) if x == y => {
                for i in 2..x + 1 {
                    walls.push(relative_wall_position(i, i));
                }
            }
            _ => unreachable!(),
        }
        walls
    }

    fn wall_positions_on_passage_from_horizontal_bound_diagonal(
        src: &Node<N>,
        dst: &Node<N>,
    ) -> ForcedVec<Position<N>, <<N as Mul<N>>::Output as Mul<U4>>::Output> {
        use AbsoluteDirection::*;
        use RelativeDirection::*;

        let relative_wall_position = Self::relative_wall_position_fn(&src, NorthEast);

        let mut walls = ForcedVec::new();
        walls.push(relative_wall_position(1, 1));
        match src.difference(dst, NorthEast) {
            (2, 1, FrontRight) => (),
            (2, 0, Right) | (2, -1, BackRight) => {
                walls.push(relative_wall_position(2, 0));
            }
            (x, y, Front) if x == y => {
                for i in 2..x + 1 {
                    walls.push(relative_wall_position(i, i));
                }
            }
            _ => unreachable!(),
        }
        walls
    }
}

impl<N, M> GraphConverter<NodeId<N>> for Maze<N, M>
where
    N: Mul<N> + Unsigned + PowerOfTwo + Debug,
    <N as Mul<N>>::Output: Mul<U2> + Mul<U4>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
    <<N as Mul<N>>::Output as Mul<U4>>::Output: ArrayLength<SearchNodeId<N>>,
    <<N as Mul<N>>::Output as Mul<U4>>::Output: ArrayLength<Node<N>>,
    <<N as Mul<N>>::Output as Mul<U4>>::Output: ArrayLength<Position<N>>,
{
    type SearchNode = SearchNodeId<N>;
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
        let mut positions =
            ForcedVec::<Position<N>, <<N as Mul<N>>::Output as Mul<U4>>::Output>::new();
        for dst in path.into_iter() {
            let dst = dst.to_node();
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
                positions.extend_from_slice(&positions_on_passage);
            }
            src = Some(dst);
        }

        let mut checker_nodes =
            ForcedVec::<Node<N>, <<N as Mul<N>>::Output as Mul<U4>>::Output>::new();
        for (&src, &dst) in positions.iter().zip(positions.iter().skip(1)) {
            if self.is_checked_by_position(dst) {
                continue;
            }
            let mut add_if_no_wall = |dx, dy, direction| {
                if !self.is_wall_by_position(dst.relative_position(dx, dy)) {
                    checker_nodes.push(dst.relative_node(dx, dy, direction));
                }
            };
            match dst.location() {
                VerticalBound => match dst.difference(&src) {
                    (-1, -1) | (-2, 0) | (-1, 1) => {
                        add_if_no_wall(-1, -1, North);
                        add_if_no_wall(-2, 0, East);
                        add_if_no_wall(-1, 1, South);
                        add_if_no_wall(0, 0, West);
                    }
                    (1, -1) | (2, 0) | (1, 1) => {
                        add_if_no_wall(1, 1, South);
                        add_if_no_wall(2, 0, West);
                        add_if_no_wall(1, -1, North);
                        add_if_no_wall(0, 0, East);
                    }
                    _ => unreachable!(),
                },
                HorizontalBound => match dst.difference(&src) {
                    (-1, -1) | (0, -2) | (1, -1) => {
                        add_if_no_wall(-1, -1, East);
                        add_if_no_wall(0, -2, North);
                        add_if_no_wall(1, -1, West);
                        add_if_no_wall(0, 0, South);
                    }
                    (-1, 1) | (0, 2) | (1, 1) => {
                        add_if_no_wall(-1, 1, East);
                        add_if_no_wall(0, 2, South);
                        add_if_no_wall(1, 1, West);
                        add_if_no_wall(0, 0, North);
                    }
                    _ => unreachable!(),
                },
                Cell => unreachable!(),
            }
        }
        let checker_nodes: Vec<Node<N>, <<N as Mul<N>>::Output as Mul<U4>>::Output> =
            checker_nodes.into();
        checker_nodes
            .into_iter()
            .filter_map(|node| node.try_into().ok())
            .collect()
    }
}

impl<N, M> NodeChecker<SearchNodeId<N>> for Maze<N, M>
where
    N: Mul<N> + Unsigned + PowerOfTwo,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
{
    //TODO: write test
    fn is_available(&self, node: &SearchNodeId<N>) -> Result<bool, CannotCheckError> {
        let position = node.to_node().position();
        if self.is_checked_by_position(position) {
            if self.is_wall_by_position(position) {
                Ok(false)
            } else {
                Ok(true)
            }
        } else {
            Err(CannotCheckError)
        }
    }
}

impl<N, M> Converter<(SearchNodeId<N>, SearchNodeId<N>)> for Maze<N, M>
where
    N: Mul<N> + Unsigned + PowerOfTwo,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
{
    type Error = ConversionError;
    type Target = SearchKind;

    //TODO: write test
    fn convert(
        &self,
        target: &(SearchNodeId<N>, SearchNodeId<N>),
    ) -> Result<Self::Target, Self::Error> {
        let direction = target
            .0
            .to_node()
            .direction()
            .relative(target.1.to_node().direction());
        Ok(SearchKind::Search(direction))
    }
}

impl<N, M> ObstacleInterpreter<Obstacle> for Maze<N, M>
where
    N: Mul<N> + Unsigned + PowerOfTwo,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
    M: Math,
{
    fn interpret_obstacles<Obstacles: IntoIterator<Item = Obstacle>>(&self, obstacles: Obstacles) {
        if let Ok(mut probs) = self.wall_existence_probs.try_borrow_mut() {
            for obstacle in obstacles {
                if let Ok(wall_info) = self.converter.convert::<N>(&obstacle) {
                    let index = wall_info.position.as_index();
                    let existence_prob = probs[index];
                    if existence_prob == 0.0 || existence_prob == 1.0 {
                        continue;
                    }
                    let exist_val = {
                        let tmp = ((wall_info.existing_distance - obstacle.distance.mean)
                            / obstacle.distance.standard_deviation)
                            .get::<ratio>();
                        -tmp * tmp / 2.0
                    };
                    let not_exist_val = {
                        let tmp = ((wall_info.not_existing_distance - obstacle.distance.mean)
                            / obstacle.distance.standard_deviation)
                            .get::<ratio>();
                        -tmp * tmp / 2.0
                    };

                    let min = if exist_val < not_exist_val {
                        exist_val
                    } else {
                        not_exist_val
                    };
                    let exist_val = M::expf(exist_val - min) * existence_prob;
                    let not_exist_val = M::expf(not_exist_val - min) * (1.0 - existence_prob);

                    probs[index] = if exist_val.is_infinite() {
                        1.0
                    } else if not_exist_val.is_infinite() {
                        0.0
                    } else {
                        exist_val / (exist_val + not_exist_val)
                    }
                }
            }
        }
    }
}

impl<N, M> Converter<SearchNodeId<N>> for Maze<N, M>
where
    N: Mul<N> + Unsigned + PowerOfTwo,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
{
    type Error = ConversionError;
    type Target = Pose;

    //TODO: write test
    fn convert(&self, node: &SearchNodeId<N>) -> Result<Self::Target, Self::Error> {
        self.converter.convert_node(node)
    }
}

impl<N, M> core::fmt::Debug for Maze<N, M>
where
    N: Mul<N> + Unsigned + PowerOfTwo + core::fmt::Debug,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        use WallDirection::*;

        writeln!(f, "Maze {{")?;
        writeln!(f, "walls:")?;

        let prob = self.wall_existence_probs.borrow();
        for y in (0..N::U16).rev() {
            for x in 0..N::U16 {
                let index = WallPosition::<N>::new(x, y, Up)
                    .unwrap_or_else(|_| unreachable!())
                    .as_index();
                write!(f, " {:1.1} --+--", prob[index])?;
            }
            writeln!(f, "")?;
            for _ in 0..N::U16 {
                write!(f, "       |  ")?;
            }
            writeln!(f, "")?;
            for x in 0..N::U16 {
                let index = WallPosition::<N>::new(x, y, Right)
                    .unwrap_or_else(|_| unreachable!())
                    .as_index();
                write!(f, "      {:1.1} ", prob[index])?;
            }
            writeln!(f, "")?;
            for _ in 0..N::U16 {
                write!(f, "       |  ")?;
            }
            writeln!(f, "")?;
        }
        Ok(())
    }
}

pub struct MazeBuilder<C, SW, WW, WPT, S, R> {
    costs: C,
    square_width: SW,
    wall_width: WW,
    wall_prob_threshold: WPT,
    init: S,
    ignore_radius: R,
}

impl<C, SW, WW, WPT, S, R> MazeBuilder<C, SW, WW, WPT, S, R> {
    const DEFAULT_SQUARE_WIDTH: Length = Length {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.09,
    };
    const DEFAULT_WALL_WIDTH: Length = Length {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.006,
    };
    const DEFAULT_PROB_THRESHOLD: f32 = 0.05;
    const DEFAULT_IGNORE_RADIUS: Length = Length {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.01,
    };
}

impl MazeBuilder<(), (), (), (), (), ()> {
    pub fn new() -> Self {
        Self {
            costs: (),
            square_width: (),
            wall_width: (),
            wall_prob_threshold: (),
            init: (),
            ignore_radius: (),
        }
    }
}

impl MazeBuilder<fn(Pattern) -> u16, (), (), (), (), ()> {
    pub fn build<N, M>(self) -> Maze<N, M>
    where
        N: Mul<N> + Unsigned + PowerOfTwo,
        <N as Mul<N>>::Output: Mul<U2>,
        <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
        M: Math,
    {
        let maze = Maze::new(
            Self::DEFAULT_PROB_THRESHOLD,
            self.costs,
            Self::DEFAULT_SQUARE_WIDTH,
            Self::DEFAULT_WALL_WIDTH,
            Self::DEFAULT_IGNORE_RADIUS,
        );
        maze.initialize();
        maze
    }
}

impl MazeBuilder<fn(Pattern) -> u16, Length, Length, (), (), Length> {
    pub fn build<N, M>(self) -> Maze<N, M>
    where
        N: Mul<N> + Unsigned + PowerOfTwo,
        <N as Mul<N>>::Output: Mul<U2>,
        <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
        M: Math,
    {
        let maze = Maze::new(
            Self::DEFAULT_PROB_THRESHOLD,
            self.costs,
            self.square_width,
            self.wall_width,
            self.ignore_radius,
        );
        maze.initialize();
        maze
    }
}

impl MazeBuilder<fn(Pattern) -> u16, (), (), f32, (), ()> {
    pub fn build<N, M>(self) -> Maze<N, M>
    where
        N: Mul<N> + Unsigned + PowerOfTwo,
        <N as Mul<N>>::Output: Mul<U2>,
        <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
        M: Math,
    {
        let maze = Maze::new(
            self.wall_prob_threshold,
            self.costs,
            Self::DEFAULT_SQUARE_WIDTH,
            Self::DEFAULT_WALL_WIDTH,
            Self::DEFAULT_IGNORE_RADIUS,
        );
        maze.initialize();
        maze
    }
}

impl MazeBuilder<fn(Pattern) -> u16, Length, Length, f32, (), Length> {
    pub fn build<N, M>(self) -> Maze<N, M>
    where
        N: Mul<N> + Unsigned + PowerOfTwo,
        <N as Mul<N>>::Output: Mul<U2>,
        <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
        M: Math,
    {
        let maze = Maze::new(
            self.wall_prob_threshold,
            self.costs,
            self.square_width,
            self.wall_width,
            self.ignore_radius,
        );
        maze.initialize();
        maze
    }
}

impl MazeBuilder<fn(Pattern) -> u16, (), (), (), &str, ()> {
    pub fn build<N, M>(self) -> Maze<N, M>
    where
        N: Mul<N> + Unsigned + PowerOfTwo,
        <N as Mul<N>>::Output: Mul<U2>,
        <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
        M: Math,
    {
        let maze = Maze::new(
            Self::DEFAULT_PROB_THRESHOLD,
            self.costs,
            Self::DEFAULT_SQUARE_WIDTH,
            Self::DEFAULT_WALL_WIDTH,
            Self::DEFAULT_IGNORE_RADIUS,
        );
        maze.initialize_with_str(self.init);
        maze
    }
}

impl MazeBuilder<fn(Pattern) -> u16, Length, Length, (), &str, Length> {
    pub fn build<N, M>(self) -> Maze<N, M>
    where
        N: Mul<N> + Unsigned + PowerOfTwo,
        <N as Mul<N>>::Output: Mul<U2>,
        <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
        M: Math,
    {
        let maze = Maze::new(
            Self::DEFAULT_PROB_THRESHOLD,
            self.costs,
            self.square_width,
            self.wall_width,
            self.ignore_radius,
        );
        maze.initialize_with_str(self.init);
        maze
    }
}

impl MazeBuilder<fn(Pattern) -> u16, (), (), f32, &str, ()> {
    pub fn build<N, M>(self) -> Maze<N, M>
    where
        N: Mul<N> + Unsigned + PowerOfTwo,
        <N as Mul<N>>::Output: Mul<U2>,
        <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
        M: Math,
    {
        let maze = Maze::new(
            self.wall_prob_threshold,
            self.costs,
            Self::DEFAULT_SQUARE_WIDTH,
            Self::DEFAULT_WALL_WIDTH,
            Self::DEFAULT_IGNORE_RADIUS,
        );
        maze.initialize_with_str(self.init);
        maze
    }
}

impl MazeBuilder<fn(Pattern) -> u16, Length, Length, f32, &str, Length> {
    pub fn build<N, M>(self) -> Maze<N, M>
    where
        N: Mul<N> + Unsigned + PowerOfTwo,
        <N as Mul<N>>::Output: Mul<U2>,
        <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
        M: Math,
    {
        let maze = Maze::new(
            self.wall_prob_threshold,
            self.costs,
            self.square_width,
            self.wall_width,
            self.ignore_radius,
        );
        maze.initialize_with_str(self.init);
        maze
    }
}

impl<SW, WW, WPT, S, R> MazeBuilder<(), SW, WW, WPT, S, R> {
    pub fn costs(
        self,
        costs: fn(Pattern) -> u16,
    ) -> MazeBuilder<fn(Pattern) -> u16, SW, WW, WPT, S, R> {
        MazeBuilder {
            wall_prob_threshold: self.wall_prob_threshold,
            costs,
            square_width: self.square_width,
            wall_width: self.wall_width,
            init: self.init,
            ignore_radius: self.ignore_radius,
        }
    }
}

impl<C, WW, WPT, S, R> MazeBuilder<C, (), WW, WPT, S, R> {
    pub fn square_width(self, square_width: Length) -> MazeBuilder<C, Length, WW, WPT, S, R> {
        MazeBuilder {
            wall_prob_threshold: self.wall_prob_threshold,
            costs: self.costs,
            square_width,
            wall_width: self.wall_width,
            init: self.init,
            ignore_radius: self.ignore_radius,
        }
    }
}

impl<C, SW, WPT, S, R> MazeBuilder<C, SW, (), WPT, S, R> {
    pub fn wall_width(self, wall_width: Length) -> MazeBuilder<C, SW, Length, WPT, S, R> {
        MazeBuilder {
            wall_prob_threshold: self.wall_prob_threshold,
            costs: self.costs,
            square_width: self.square_width,
            wall_width,
            init: self.init,
            ignore_radius: self.ignore_radius,
        }
    }
}

impl<C, SW, WW, S, R> MazeBuilder<C, SW, WW, (), S, R> {
    ///NOTE: This value should be in [0.0,0.5].
    pub fn wall_existence_probability_threshold(
        self,
        wall_prob_threshold: f32,
    ) -> MazeBuilder<C, SW, WW, f32, S, R> {
        assert!(wall_prob_threshold >= 0.0 && wall_prob_threshold <= 0.5);
        MazeBuilder {
            wall_prob_threshold,
            costs: self.costs,
            square_width: self.square_width,
            wall_width: self.wall_width,
            init: self.init,
            ignore_radius: self.ignore_radius,
        }
    }
}

impl<C, SW, WW, WPT, R> MazeBuilder<C, SW, WW, WPT, (), R> {
    ///example of maze format:
    ///```maze
    ///+---+---+
    ///|   |   |
    ///+---+   +
    ///|       |
    ///+---+---+
    ///```
    pub fn init_with(self, init: &str) -> MazeBuilder<C, SW, WW, WPT, &str, R> {
        MazeBuilder {
            wall_prob_threshold: self.wall_prob_threshold,
            costs: self.costs,
            square_width: self.square_width,
            wall_width: self.wall_width,
            init,
            ignore_radius: self.ignore_radius,
        }
    }
}

impl<C, SW, WW, WPT, S> MazeBuilder<C, SW, WW, WPT, S, ()> {
    pub fn ignore_radius_from_pillar(
        self,
        ignore_radius: Length,
    ) -> MazeBuilder<C, SW, WW, WPT, S, Length> {
        MazeBuilder {
            wall_prob_threshold: self.wall_prob_threshold,
            costs: self.costs,
            square_width: self.square_width,
            wall_width: self.wall_width,
            init: self.init,
            ignore_radius,
        }
    }
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;
    use uom::si::{angle::degree, f32::*, length::meter};

    use super::*;
    use crate::utils::math::MathFake;
    use crate::utils::sample::Sample;

    fn cost(pattern: Pattern) -> u16 {
        use Pattern::*;

        match pattern {
            Straight(length) => length,
            StraightDiagonal(length) => 2 * length,
            Search90 => 3,
            FastRun45 => 4,
            FastRun90 => 5,
            FastRun135 => 6,
            FastRun180 => 7,
            FastRunDiagonal90 => 8,
            SpinBack => 9,
        }
    }

    mod successors_predecessors_tests {
        macro_rules! node_tests {
            ($($name:ident: ($method: ident, $value: expr),)*) => {
                $(
                    #[test]
                    fn $name() {
                        use super::*;
                        use AbsoluteDirection::*;
                        use Pattern::*;

                        fn new(x: u16, y: u16, direction: AbsoluteDirection) -> NodeId<U4> {
                            NodeId::<U4>::new(x, y, direction).unwrap()
                        }

                        let maze = MazeBuilder::new().costs(cost).build::<U4, MathFake>();
                        let (input, mut expected) = $value;
                        expected.sort();
                        let mut nodes = maze.$method(&input);
                        nodes.sort();
                        assert_eq!(nodes, expected.as_slice());
                    }
                )*
            }
        }

        node_tests! {
            test_node_predecessors1: (predecessors, (
                new(2, 2, North),
                vec![
                    (new(2, 0, North), cost(Straight(1))),
                    (new(3, 0, NorthWest), cost(FastRun45)),
                    (new(4, 0, West), cost(FastRun90)),
                    (new(4, 1, SouthWest), cost(FastRun135)),
                    (new(4, 2, South), cost(FastRun180)),
                ],
            )),
            test_node_predecessors2: (predecessors, (
                new(3, 2, NorthEast),
                vec![
                    (new(2, 1, NorthEast), cost(StraightDiagonal(1))),
                    (new(2, 0, North), cost(FastRun45)),
                    (new(4, 0, West), cost(FastRun135)),
                    (new(3, 0, NorthWest), cost(FastRunDiagonal90)),
                ],
            )),
            test_node_predecessors3: (predecessors, (
                new(2, 3, NorthEast),
                vec![
                    (new(1, 2, NorthEast), cost(StraightDiagonal(1))),
                    (new(0, 1, NorthEast), cost(StraightDiagonal(2))),
                    (new(0, 2, East), cost(FastRun45)),
                    (new(0, 4, South), cost(FastRun135)),
                    (new(0, 3, SouthEast), cost(FastRunDiagonal90)),
                ],
            )),
            test_node_successors1: (successors, (
                new(2, 0, North),
                vec![
                    (new(2, 2, North), cost(Straight(1))),
                    (new(2, 4, North), cost(Straight(2))),
                    (new(2, 6, North), cost(Straight(3))),
                    (new(3, 2, NorthEast), cost(FastRun45)),
                    (new(4, 2, East), cost(FastRun90)),
                    (new(4, 1, SouthEast), cost(FastRun135)),
                    (new(4, 0, South), cost(FastRun180)),
                    (new(1, 2, NorthWest), cost(FastRun45)),
                    (new(0, 2, West), cost(FastRun90)),
                    (new(0, 1, SouthWest), cost(FastRun135)),
                    (new(0, 0, South), cost(FastRun180)),
                ],
            )),
            test_node_successors2: (successors, (
                new(0, 2, East),
                vec![
                    (new(2, 2, East), cost(Straight(1))),
                    (new(4, 2, East), cost(Straight(2))),
                    (new(6, 2, East), cost(Straight(3))),
                    (new(2, 1, SouthEast), cost(FastRun45)),
                    (new(2, 0, South), cost(FastRun90)),
                    (new(2, 3, NorthEast), cost(FastRun45)),
                    (new(2, 4, North), cost(FastRun90)),
                    (new(1, 4, NorthWest), cost(FastRun135)),
                    (new(0, 4, West), cost(FastRun180)),
                ],
            )),
            test_node_successors3: (successors, (
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
            )),
            test_node_successors4: (successors, (
                new(2, 3, SouthWest),
                vec![
                    (new(1, 2, SouthWest), cost(StraightDiagonal(1))),
                    (new(0, 1, SouthWest), cost(StraightDiagonal(2))),
                    (new(0, 2, West), cost(FastRun45)),
                    (new(0, 4, North), cost(FastRun135)),
                    (new(0, 3, NorthWest), cost(FastRunDiagonal90)),
                ],
            )),
            test_node_successors5: (successors, (
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
            )),
        }
    }

    #[test]
    fn test_search_node_successors() {
        use AbsoluteDirection::*;
        use Pattern::*;

        let maze = MazeBuilder::new().costs(cost).build::<U4, MathFake>();

        let new = |x, y, direction| SearchNodeId::<U4>::new(x, y, direction).unwrap();

        let test_data = vec![
            (
                new(2, 1, North),
                vec![
                    (new(2, 1, South), cost(SpinBack)),
                    (new(3, 2, East), cost(Search90)),
                    (new(1, 2, West), cost(Search90)),
                    (new(2, 3, North), cost(Straight(1))),
                ],
            ),
            (
                new(1, 2, East),
                vec![
                    (new(1, 2, West), cost(SpinBack)),
                    (new(2, 1, South), cost(Search90)),
                    (new(2, 3, North), cost(Search90)),
                    (new(3, 2, East), cost(Straight(1))),
                ],
            ),
        ];

        for (src, mut expected) in test_data {
            expected.sort();
            let mut successors = maze.successors(&src);
            successors.sort();
            assert_eq!(successors, expected.as_slice());
        }
    }

    #[test]
    fn test_seach_node_predecessors() {
        use AbsoluteDirection::*;
        use Pattern::*;

        let maze = MazeBuilder::new().costs(cost).build::<U4, MathFake>();

        let new = |x, y, direction| SearchNodeId::<U4>::new(x, y, direction).unwrap();

        let test_data = vec![
            (
                new(2, 3, North),
                vec![
                    (new(2, 3, South), cost(SpinBack)),
                    (new(3, 2, West), cost(Search90)),
                    (new(1, 2, East), cost(Search90)),
                    (new(2, 1, North), cost(Straight(1))),
                ],
            ),
            (
                new(3, 2, East),
                vec![
                    (new(3, 2, West), cost(SpinBack)),
                    (new(2, 1, North), cost(Search90)),
                    (new(2, 3, South), cost(Search90)),
                    (new(1, 2, East), cost(Straight(1))),
                ],
            ),
        ];

        for (src, mut expected) in test_data {
            expected.sort();
            let mut predecessors = maze.predecessors(&src);
            predecessors.sort();
            assert_eq!(predecessors, expected.as_slice());
        }
    }

    macro_rules! convert_to_checker_nodes_tests {
        ($($name: ident: $value: expr,)*) => {
            $(
                #[test]
                fn $name() {
                    use AbsoluteDirection::*;
                    use WallDirection::*;
                    use std::vec::Vec;

                    let new = |x, y, direction| NodeId::<U4>::new(x, y, direction).unwrap();
                    let new_search = |x, y, direction| SearchNodeId::<U4>::new(x, y, direction).unwrap();
                    let new_wall = |x, y, z| WallPosition::new(x, y, z).unwrap();

                    let (walls, path, expected) = $value;
                    let walls = walls.into_iter().map(|(x,y,z)| new_wall(x,y,z)).collect::<Vec<_>>();
                    let path = path.into_iter().map(|(x,y,dir)| new(x,y,dir)).collect::<Vec<_>>();
                    let mut expected = expected.into_iter().map(|(x,y,dir)| new_search(x,y,dir)).collect::<Vec<_>>();

                    let maze = MazeBuilder::new().costs(cost).build::<U4, MathFake>();
                    for wall in walls {
                        maze.check_wall(wall, true);
                    }
                    expected.sort();
                    let mut checker_nodes = maze.convert_to_checker_nodes(path);
                    checker_nodes.sort();
                    assert_eq!(checker_nodes, expected.as_slice());
                }
            )*
        }
    }

    convert_to_checker_nodes_tests! {
        test_convert_to_checker_nodes1:
            (
                vec![
                    (0, 0, Right),
                    (0, 1, Right),
                    (0, 2, Up),
                    (1, 0, Up),
                    (1, 1, Up),
                    (1, 2, Right),
                    (2, 1, Right),
                    (2, 2, Up),
                    (3, 1, Up),
                ],
                vec![
                    (0, 0, North),
                    (0, 2, North),
                    (1, 4, NorthEast),
                    (2, 5, NorthEast),
                    (4, 6, East),
                    (5, 4, SouthWest),
                    (4, 2, South),
                    (2, 0, West),
                ],
                vec![
                    (0, 1, North),
                    (0, 3, South),
                    (0, 3, North),
                    (1, 4, East),
                    (1, 4, West),
                    (2, 5, South),
                    (2, 5, North),
                    (1, 6, East),
                    (3, 6, East),
                    (3, 6, West),
                    (5, 6, East),
                    (5, 6, West),
                    (6, 5, South),
                    (6, 5, North),
                    (5, 4, East),
                    (5, 4, West),
                    (4, 3, North),
                    (4, 3, South),
                    (4, 1, North),
                    (4, 1, South),
                    (3, 2, East),
                    (5, 0, West),
                    (3, 0, East),
                ],
            ),
        test_convert_to_checker_nodes2:
            (
                vec![(1, 2, Right)],
                vec![
                    (0, 0, North),
                    (1, 2, NorthEast),
                    (2, 4, North),
                    (4, 4, South),
                ],
                vec![
                    (0, 1, North),
                    (0, 3, South),
                    (1, 2, West),
                    (1, 2, East),
                    (2, 1, North),
                    (3, 2, West),
                    (2, 3, South),
                    (2, 3, North),
                    (1, 4, East),
                    (2, 5, South),
                    (2, 5, North),
                    (1, 6, East),
                    (3, 6, West),
                    (3, 6, East),
                    (5, 6, West),
                    (4, 5, North),
                ],
            ),
    }

    proptest! {
        #[test]
        fn test_obstacle_interpreter(
            x in 0.0f32..0.36,
            y in 0.0f32..0.36,
            theta in 0.0f32..360.0,
            mean_distance in 0.0f32..0.3,
            standard_deviation in 0.001f32..0.05,
        ) {
            use core::ops::Deref;

            let maze = MazeBuilder::new().costs(cost).build::<U4, MathFake>();
            let obstacle = Obstacle {
                source: Pose {
                    x: Length::new::<meter>(x),
                    y: Length::new::<meter>(y),
                    theta: Angle::new::<degree>(theta),
                },
                distance: Sample {
                    mean: Length::new::<meter>(mean_distance),
                    standard_deviation: Length::new::<meter>(standard_deviation),
                }
            };
            maze.interpret_obstacles(vec![obstacle]);
            for prob in maze.wall_existence_probs().borrow().deref() {
                prop_assert!(!prob.is_nan())
            }
        }
    }

    #[test]
    fn test_initialize_with_str() {
        let input = "+---+---+
|   |   |
+---+   +
|       |
+---+---+";
        let maze = MazeBuilder::new()
            .costs(cost)
            .init_with(input)
            .build::<U2, MathFake>();

        let test_cases = vec![
            ((1, 0), false),
            ((3, 0), true),
            ((0, 1), true),
            ((2, 1), false),
            ((1, 2), true),
            ((3, 2), true),
            ((0, 3), true),
            ((2, 3), true),
        ];

        for (position, expected) in test_cases {
            let position = Position::new(position.0, position.1);
            assert_eq!(
                maze.is_wall_by_position(position),
                expected,
                "{:?}",
                position
            );
        }
    }
}
