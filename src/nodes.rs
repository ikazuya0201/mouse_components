//! Implementations of nodes.

mod direction;

use core::marker::PhantomData;
use core::ops::Mul;

use heapless::{ArrayLength, Vec};
use serde::{Deserialize, Serialize};
use typenum::{consts::*, PowerOfTwo, Unsigned};

use crate::commanders::{NextNode, RotationNode, RouteNode};
use crate::mazes::NeighborNumberUpperBound;
use crate::mazes::{GraphNode, WallFinderNode, WallNode, WallSpaceNode};
use crate::trajectory_generators::{RunKind, SearchKind, SlalomDirection, SlalomKind};
use crate::utils::forced_vec::ForcedVec;
use crate::wall_manager::Wall;
pub use direction::{AbsoluteDirection, RelativeDirection};

#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
/// An enum that represents the type of edges of several graphs.
pub enum Pattern {
    Straight(u8),
    StraightDiagonal(u8),
    Search90,
    FastRun45,
    FastRun90,
    FastRun135,
    FastRun180,
    FastRunDiagonal90,
    SpinBack,
}

//TODO: Create new data type to reduce copy cost.
#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
/// A type that is the super set of [SearchNode] and [RunNode].
pub struct Node<N> {
    x: i8,
    y: i8,
    direction: AbsoluteDirection,
    _maze_width: PhantomData<fn() -> N>,
}

impl<N> NextNode<SearchKind> for Node<N>
where
    N: Unsigned,
{
    type Error = NodeCreationError;

    fn next(&self, route: &SearchKind) -> Result<Self, Self::Error> {
        use AbsoluteDirection::*;
        use RelativeDirection::*;

        match route {
            SearchKind::Init | SearchKind::Final => self.relative(0, 1, Front, North),
            SearchKind::Front => self.relative(0, 2, Front, North),
            SearchKind::Right => self.relative(1, 1, Right, North),
            SearchKind::Left => self.relative(-1, 1, Left, North),
            SearchKind::Back => self.relative(0, 0, Back, North),
        }
    }
}

impl<N> core::fmt::Debug for Node<N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "Node{{ x:{}, y:{}, direction:{:?} }}",
            self.x, self.y, self.direction
        )
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
/// Error on [Node], [SearchNode] and [RunNode].
pub struct NodeCreationError {
    x: i8,
    y: i8,
    direction: AbsoluteDirection,
}

#[derive(Clone, PartialEq, Eq, Debug)]
struct RelativeWallError {
    x: i8,
    y: i8,
    direction: AbsoluteDirection,
    dx: i8,
    dy: i8,
    base_dir: AbsoluteDirection,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Location {
    Cell,
    VerticalBound,
    HorizontalBound,
    Pillar,
}

impl<N> Node<N> {
    unsafe fn new_unchecked(x: i8, y: i8, direction: AbsoluteDirection) -> Self {
        Self {
            x,
            y,
            direction,
            _maze_width: PhantomData,
        }
    }

    pub fn x(&self) -> &i8 {
        &self.x
    }

    pub fn y(&self) -> &i8 {
        &self.y
    }

    pub fn direction(&self) -> &AbsoluteDirection {
        &self.direction
    }

    fn location(&self) -> Location {
        use Location::*;

        if self.x & 1 == 0 {
            if self.y & 1 == 0 {
                Cell
            } else {
                HorizontalBound
            }
        } else if self.y & 1 == 0 {
            VerticalBound
        } else {
            Pillar
        }
    }

    fn difference(&self, other: &Self, base_dir: AbsoluteDirection) -> (i8, i8, RelativeDirection) {
        use RelativeDirection::*;

        let dx = other.x - self.x;
        let dy = other.y - self.y;
        let (dx, dy) = match base_dir.relative(self.direction) {
            Front => (dx, dy),
            Right => (-dy, dx),
            Back => (-dx, -dy),
            Left => (dy, -dx),
            _ => unreachable!(),
        };
        (dx, dy, self.direction.relative(other.direction))
    }
}

impl<N> Node<N>
where
    N: Unsigned,
{
    fn new(x: i8, y: i8, direction: AbsoluteDirection) -> Result<Self, NodeCreationError> {
        if x < 0 || y < 0 || x > Self::max() || y > Self::max() {
            Err(NodeCreationError { x, y, direction })
        } else {
            Ok(unsafe { Self::new_unchecked(x, y, direction) })
        }
    }

    fn max() -> i8 {
        2 * N::I8 - 1
    }

    fn relative(
        &self,
        dx: i8,
        dy: i8,
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
        Self::new(self.x + dx, self.y + dy, self.direction.rotate(ddir))
    }
}

impl<N> Node<N>
where
    N: Unsigned + PowerOfTwo,
{
    fn relative_wall(
        &self,
        dx: i8,
        dy: i8,
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

        let x = x as u8;
        let y = y as u8;
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

//TODO: Remove Copy
#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
/// A type that represents nodes (verteces) of a graph for search.
pub struct SearchNode<N>(Node<N>);

impl<N> SearchNode<N> {
    pub fn inner(&self) -> &Node<N> {
        &self.0
    }

    pub fn into_inner(self) -> Node<N> {
        self.0
    }
}

impl<N> From<SearchNode<N>> for Node<N> {
    fn from(value: SearchNode<N>) -> Self {
        value.0
    }
}

impl<N> core::fmt::Debug for SearchNode<N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "RunNode({:?})", self.0)
    }
}

impl<N> SearchNode<N> {
    /// Creates a new `SearchNode` from the given values.
    ///
    /// # Safety
    ///
    /// The position must be a bound of two squares (the evenness of the `x` and the `y` is different).
    /// When the position is a vertical bound (the `x` is odd and the `y` is even), the `direction` must be
    /// `East` or `West`.
    /// Otherwise, the `direction` must be `North` or `South`.
    #[inline]
    #[must_use]
    pub unsafe fn new_unchecked(x: i8, y: i8, direction: AbsoluteDirection) -> Self {
        Self(Node::<N>::new_unchecked(x, y, direction))
    }

    #[inline]
    fn is_valid_direction(x: i8, y: i8, direction: AbsoluteDirection) -> bool {
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
    N: Unsigned + PowerOfTwo,
{
    #[inline]
    fn y_offset() -> u32 {
        (N::USIZE * 2).trailing_zeros()
    }

    #[inline]
    fn direction_offset() -> u32 {
        2 * (N::USIZE * 2).trailing_zeros()
    }
}

impl<N> Into<usize> for SearchNode<N>
where
    N: Unsigned + PowerOfTwo,
{
    fn into(self) -> usize {
        use AbsoluteDirection::*;

        let direction = if self.0.x() & 1 == 0 {
            if self.0.y() & 1 == 0 {
                unreachable!()
            } else {
                match self.0.direction() {
                    North => 0,
                    South => 1,
                    _ => unreachable!(),
                }
            }
        } else {
            if self.0.y() & 1 == 0 {
                match self.0.direction() {
                    East => 0,
                    West => 1,
                    _ => unreachable!(),
                }
            } else {
                unreachable!()
            }
        };
        *self.0.x() as usize
            | ((*self.0.y() as usize) << Self::y_offset())
            | (direction << Self::direction_offset())
    }
}

impl<N> From<SearchNode<N>> for Wall<N> {
    fn from(value: SearchNode<N>) -> Self {
        unsafe {
            Wall::new_unchecked(
                *value.0.x() as u8 / 2,
                *value.0.y() as u8 / 2,
                value.0.x() & 1 == 0,
            )
        }
    }
}

impl<N> SearchNode<N>
where
    N: Unsigned,
{
    /// Creates a new `SearchNode` with a check of constraints.
    pub fn new(x: i8, y: i8, direction: AbsoluteDirection) -> Result<Self, NodeCreationError> {
        use core::convert::TryInto;
        Node::<N>::new(x, y, direction)?.try_into()
    }

    fn relative(
        &self,
        dx: i8,
        dy: i8,
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
        let mut update = |dx: i8, dy: i8, ddir: RelativeDirection, pattern: Pattern| {
            use AbsoluteDirection::*;
            let (dx, dy) = if succs { (dx, dy) } else { (-dx, -dy) };
            if let Ok(wall) = self.0.relative_wall(dx, dy, North) {
                let mut wall_nodes = ForcedVec::new();
                wall_nodes.push(WallNode::Wall(wall));
                wall_nodes.push(WallNode::Node((
                    self.relative(dx, dy, ddir, North)
                        .expect("Should never panic!"),
                    pattern,
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
    type Pattern = Pattern;
    type WallNodes = Vec<WallNode<Self::Wall, (Self, Self::Pattern)>, U2>;
    type WallNodesList = Vec<Self::WallNodes, U4>;

    fn successors(&self) -> Self::WallNodesList {
        self.neighbors(true)
    }

    fn predecessors(&self) -> Self::WallNodesList {
        self.neighbors(false)
    }
}

#[derive(Clone, PartialEq, Eq, Debug)]
/// Error on the implementation of [RouteNode] for node types.
pub struct RouteError<T> {
    src: T,
    dst: T,
}

impl<N: Clone> RouteNode for SearchNode<N> {
    type Error = RouteError<SearchNode<N>>;
    type Route = SearchKind;

    fn route(&self, to: &Self) -> Result<Self::Route, Self::Error> {
        use RelativeDirection::*;

        Ok(match self.0.difference(&to.0, AbsoluteDirection::North) {
            (1, 1, Right) => SearchKind::Right,
            (0, 2, Front) => SearchKind::Front,
            (-1, 1, Left) => SearchKind::Left,
            (0, 0, Back) => SearchKind::Back,
            _ => {
                return Err(RouteError {
                    src: self.clone(),
                    dst: to.clone(),
                })
            }
        })
    }
}

impl<N: Clone> RouteNode for RunNode<N> {
    type Error = RouteError<RunNode<N>>;
    type Route = RunKind;

    fn route(&self, to: &Self) -> Result<Self::Route, Self::Error> {
        use AbsoluteDirection::*;
        use Location::*;
        use RelativeDirection::*;
        use RunKind::*;
        use SlalomDirection::{Left as SLeft, Right as SRight};
        use SlalomKind::*;

        let create_error = || {
            Err(RouteError {
                src: self.clone(),
                dst: to.clone(),
            })
        };

        Ok(match self.0.location() {
            Cell => match self.0.difference(&to.0, North) {
                (1, 2, FrontRight) => Slalom(FastRun45, SRight),
                (-1, 2, FrontLeft) => Slalom(FastRun45, SLeft),
                (2, 2, Right) => Slalom(FastRun90, SRight),
                (-2, 2, Left) => Slalom(FastRun90, SLeft),
                (2, 1, BackRight) => Slalom(FastRun135, SRight),
                (-2, 1, BackLeft) => Slalom(FastRun135, SLeft),
                (2, 0, Back) => Slalom(FastRun180, SRight),
                (-2, 0, Back) => Slalom(FastRun180, SLeft),
                (0, x, Front) if x > 0 => Straight(x as u8 / 2),
                _ => return create_error(),
            },
            HorizontalBound | VerticalBound => match self.0.difference(&to.0, NorthEast) {
                (2, 1, FrontRight) => Slalom(FastRun45Rev, SRight),
                (2, 0, Right) => Slalom(FastRunDiagonal90, SRight),
                (2, -1, BackRight) => Slalom(FastRun135Rev, SRight),
                (1, 2, FrontLeft) => Slalom(FastRun45Rev, SLeft),
                (0, 2, Left) => Slalom(FastRunDiagonal90, SLeft),
                (-1, 2, BackLeft) => Slalom(FastRun135Rev, SLeft),
                (x, y, Front) if x == y && x > 0 => StraightDiagonal(x as u8),
                _ => return create_error(),
            },
            _ => unreachable!("Should never be on the pillar"),
        })
    }
}

//TODO: Create new data type to reduce copy cost.
#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
/// A type that represents nodes (verteces) of a graph for fast run.
pub struct RunNode<N>(Node<N>);

impl<N> RunNode<N> {
    pub fn inner(&self) -> &Node<N> {
        &self.0
    }

    pub fn into_inner(self) -> Node<N> {
        self.0
    }
}

impl<N> From<RunNode<N>> for Node<N> {
    fn from(value: RunNode<N>) -> Self {
        value.0
    }
}

impl<N> core::fmt::Debug for RunNode<N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "RunNode({:?})", self.0)
    }
}

impl<N> Into<usize> for RunNode<N>
where
    N: Unsigned + PowerOfTwo,
{
    fn into(self) -> usize {
        use AbsoluteDirection::*;

        let direction = if (self.0.x ^ self.0.y) & 1 == 1 {
            match self.0.direction {
                NorthEast => 0,
                SouthEast => 1,
                SouthWest => 2,
                NorthWest => 3,
                _ => unreachable!(),
            }
        } else if (self.0.x | self.0.y) & 1 == 0 {
            match self.0.direction {
                North => 0,
                East => 1,
                South => 2,
                West => 3,
                _ => unreachable!(),
            }
        } else {
            unreachable!()
        };

        self.0.x as usize
            | ((self.0.y as usize) << Self::y_offset())
            | (direction << Self::direction_offset())
    }
}

impl<N> RunNode<N>
where
    N: Unsigned + PowerOfTwo,
{
    #[inline]
    fn y_offset() -> u32 {
        (N::USIZE * 2).trailing_zeros()
    }

    #[inline]
    fn direction_offset() -> u32 {
        2 * (N::USIZE * 2).trailing_zeros()
    }
}

impl<N> RunNode<N> {
    /// Creates a new `RunNode` from the given values.
    ///
    /// # Safety
    ///
    /// The position must not be a pillar (both the `x` and the `y` are odd).
    /// When the position is a bound of two squares (the evenness of the `x` and the `y` is different), the
    /// `direction` must be `NorthEast`, `SouthEast`, `SouthWest` or `NorthWest`.
    /// When the position is the center of a square (both the `x` and the `y` are even), the
    /// `direction` must be `North`, `East`, `South` or `West`.
    #[inline]
    #[must_use]
    pub unsafe fn new_unchecked(x: i8, y: i8, direction: AbsoluteDirection) -> Self {
        Self(Node::<N>::new_unchecked(x, y, direction))
    }

    #[inline]
    fn is_valid_direction(x: i8, y: i8, direction: AbsoluteDirection) -> bool {
        use AbsoluteDirection::*;

        if (x ^ y) & 1 == 1 {
            //on a wall
            match direction {
                NorthEast | SouthEast | SouthWest | NorthWest => true,
                _ => false,
            }
        } else if (x | y) & 1 == 0 {
            match direction {
                North | East | South | West => true,
                _ => false,
            }
        } else {
            false
        }
    }
}

impl<N> RunNode<N>
where
    N: Unsigned,
{
    /// Creates a new `RunNode` with a check of constraints.
    pub fn new(x: i8, y: i8, direction: AbsoluteDirection) -> Result<Self, NodeCreationError> {
        use core::convert::TryInto;
        Node::<N>::new(x, y, direction)?.try_into()
    }

    fn relative(
        &self,
        dx: i8,
        dy: i8,
        ddir: RelativeDirection,
        base_dir: AbsoluteDirection,
    ) -> Result<Self, NodeCreationError> {
        use core::convert::TryInto;
        self.0.relative(dx, dy, ddir, base_dir)?.try_into()
    }
}

impl<N> RunNode<N>
where
    N: Unsigned + PowerOfTwo,
{
    fn neighbors(&self, is_succ: bool) -> <Self as GraphNode>::WallNodesList {
        use AbsoluteDirection::*;
        use Location::*;

        match self.0.location() {
            Cell => self.cell_neighbors(is_succ),
            HorizontalBound => match self.0.direction {
                NorthEast | SouthWest => self.right_diagonal_neighbors(is_succ),
                NorthWest | SouthEast => self.left_diagonal_neighbors(is_succ),
                _ => unreachable!(),
            },
            VerticalBound => match self.0.direction {
                NorthEast | SouthWest => self.left_diagonal_neighbors(is_succ),
                NorthWest | SouthEast => self.right_diagonal_neighbors(is_succ),
                _ => unreachable!(),
            },
            _ => unreachable!(),
        }
    }

    fn add_wall_fn<'a>(
        &'a self,
        is_succ: bool,
        base_dir: AbsoluteDirection,
    ) -> impl 'a
           + Fn(
        &mut ForcedVec<WallNode<Wall<N>, (RunNode<N>, Pattern)>, NeighborNumberUpperBound>,
        i8,
        i8,
    ) -> Result<(), ()> {
        move |list: &mut ForcedVec<
            WallNode<Wall<N>, (RunNode<N>, Pattern)>,
            NeighborNumberUpperBound,
        >,
              dx: i8,
              dy: i8| {
            let (dx, dy) = if is_succ { (dx, dy) } else { (-dx, -dy) };
            let wall = self.0.relative_wall(dx, dy, base_dir).map_err(|_| ())?;
            list.push(WallNode::Wall(wall));
            Ok(())
        }
    }

    fn add_node_fn<'a>(
        &'a self,
        is_succ: bool,
        base_dir: AbsoluteDirection,
    ) -> impl 'a
           + Fn(
        &mut ForcedVec<WallNode<Wall<N>, (RunNode<N>, Pattern)>, NeighborNumberUpperBound>,
        i8,
        i8,
        RelativeDirection,
        Pattern,
    ) -> Result<(), ()> {
        move |list: &mut ForcedVec<
            WallNode<Wall<N>, (RunNode<N>, Pattern)>,
            NeighborNumberUpperBound,
        >,
              dx: i8,
              dy: i8,
              ddir: RelativeDirection,
              pattern: Pattern| {
            let (dx, dy) = if is_succ { (dx, dy) } else { (-dx, -dy) };
            let node = self.relative(dx, dy, ddir, base_dir).map_err(|_| ())?;
            list.push(WallNode::Node((node, pattern)));
            Ok(())
        }
    }

    fn cell_neighbors(&self, is_succ: bool) -> <Self as GraphNode>::WallNodesList {
        use AbsoluteDirection::*;
        use Pattern::*;
        use RelativeDirection::*;

        let mut list = ForcedVec::new();

        let add_wall = self.add_wall_fn(is_succ, North);
        let add_node = self.add_node_fn(is_succ, North);

        //right slalom
        let mut tmp = ForcedVec::new();
        let _: Result<(), ()> = (|| {
            add_wall(&mut tmp, 0, 1)?;
            add_wall(&mut tmp, 1, 2)?;
            add_node(&mut tmp, 1, 2, FrontRight, FastRun45)?;
            add_node(&mut tmp, 2, 2, Right, FastRun90)?;
            add_wall(&mut tmp, 2, 1)?;
            add_node(&mut tmp, 2, 1, BackRight, FastRun135)?;
            add_node(&mut tmp, 2, 0, Back, FastRun180)?;
            Ok(())
        })();
        list.push(tmp.into());

        //left slalom
        let mut tmp = ForcedVec::new();
        let _: Result<(), ()> = (|| {
            add_wall(&mut tmp, 0, 1)?;
            add_wall(&mut tmp, -1, 2)?;
            add_node(&mut tmp, -1, 2, FrontLeft, FastRun45)?;
            add_node(&mut tmp, -2, 2, Left, FastRun90)?;
            add_wall(&mut tmp, -2, 1)?;
            add_node(&mut tmp, -2, 1, BackLeft, FastRun135)?;
            add_node(&mut tmp, -2, 0, Back, FastRun180)?;
            Ok(())
        })();
        list.push(tmp.into());

        //straight
        let mut tmp = ForcedVec::new();
        let _: Result<(), ()> = (|| {
            for dy in 1.. {
                add_wall(&mut tmp, 0, 2 * dy - 1)?;
                add_node(&mut tmp, 0, 2 * dy, Front, Straight(dy as u8))?;
            }
            Ok(())
        })();
        list.push(tmp.into());

        list.into()
    }

    fn left_diagonal_neighbors(&self, is_succ: bool) -> <Self as GraphNode>::WallNodesList {
        use AbsoluteDirection::*;
        use Pattern::*;
        use RelativeDirection::*;

        let mut list = ForcedVec::new();

        let add_node = self.add_node_fn(is_succ, NorthEast);
        let add_wall = self.add_wall_fn(is_succ, NorthEast);

        let mut tmp = ForcedVec::new();
        let _: Result<(), ()> = (|| {
            add_wall(&mut tmp, 1, 1)?;
            add_node(&mut tmp, 1, 2, FrontLeft, FastRun45)?;
            add_wall(&mut tmp, 0, 2)?;
            add_node(&mut tmp, 0, 2, Left, FastRunDiagonal90)?;
            add_node(&mut tmp, -1, 2, BackLeft, FastRun135)?;
            Ok(())
        })();
        list.push(tmp.into());

        let mut tmp = ForcedVec::new();
        let _: Result<(), ()> = (|| {
            for i in 1.. {
                add_wall(&mut tmp, i, i)?;
                add_node(&mut tmp, i, i, Front, StraightDiagonal(i as u8))?;
            }
            Ok(())
        })();
        list.push(tmp.into());

        list.into()
    }

    fn right_diagonal_neighbors(&self, is_succ: bool) -> <Self as GraphNode>::WallNodesList {
        use AbsoluteDirection::*;
        use Pattern::*;
        use RelativeDirection::*;

        let mut list = ForcedVec::new();

        let add_node = self.add_node_fn(is_succ, NorthEast);
        let add_wall = self.add_wall_fn(is_succ, NorthEast);

        let mut tmp = ForcedVec::new();
        let _: Result<(), ()> = (|| {
            add_wall(&mut tmp, 1, 1)?;
            add_node(&mut tmp, 2, 1, FrontRight, FastRun45)?;
            add_wall(&mut tmp, 2, 0)?;
            add_node(&mut tmp, 2, 0, Right, FastRunDiagonal90)?;
            add_node(&mut tmp, 2, -1, BackRight, FastRun135)?;
            Ok(())
        })();
        list.push(tmp.into());

        let mut tmp = ForcedVec::new();
        let _: Result<(), ()> = (|| {
            for i in 1.. {
                add_wall(&mut tmp, i, i)?;
                add_node(&mut tmp, i, i, Front, StraightDiagonal(i as u8))?;
            }
            Ok(())
        })();
        list.push(tmp.into());

        list.into()
    }
}

impl<N: Unsigned> core::convert::TryFrom<Node<N>> for RunNode<N> {
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

impl<N> WallSpaceNode for RunNode<N> {
    type Wall = Wall<N>;
}

//TODO: use iterator instead of vec
impl<N> GraphNode for RunNode<N>
where
    N: Unsigned + PowerOfTwo,
{
    type Pattern = Pattern;
    type WallNodes = Vec<WallNode<Self::Wall, (Self, Self::Pattern)>, NeighborNumberUpperBound>;
    type WallNodesList = Vec<Self::WallNodes, U3>;

    fn successors(&self) -> Self::WallNodesList {
        self.neighbors(true)
    }

    fn predecessors(&self) -> Self::WallNodesList {
        self.neighbors(false)
    }
}

impl<N> WallFinderNode for RunNode<N>
where
    N: Unsigned + PowerOfTwo + Mul<U2>,
    N::Output: ArrayLength<Wall<N>>,
{
    type Walls = Vec<Self::Wall, N::Output>;

    fn walls_between(&self, other: &Self) -> Self::Walls {
        use AbsoluteDirection::*;
        use Location::*;
        use RelativeDirection::*;

        let mut walls = ForcedVec::new();
        match (self.0.location(), self.0.direction()) {
            (Cell, _) => {
                let mut add = |x: i8, y: i8| {
                    if let Ok(wall) = self.0.relative_wall(x, y, North) {
                        walls.push(wall);
                    }
                };
                match self.0.difference(&other.0, North) {
                    (1, 2, FrontRight) | (2, 2, Right) => {
                        add(0, 1);
                        add(1, 2);
                    }
                    (2, 1, BackRight) | (2, 0, Back) => {
                        add(0, 1);
                        add(1, 2);
                        add(2, 1);
                    }
                    (-1, 2, FrontLeft) | (-2, 2, Left) => {
                        add(0, 1);
                        add(-1, 2);
                    }
                    (-2, 1, BackRight) | (-2, 0, Back) => {
                        add(0, 1);
                        add(-1, 2);
                        add(-2, 1);
                    }
                    (0, y, Front) => {
                        for dy in 0..y / 2 {
                            add(0, 2 * dy + 1);
                        }
                    }
                    _ => unreachable!(),
                }
            }
            (HorizontalBound, NorthEast)
            | (HorizontalBound, SouthWest)
            | (VerticalBound, SouthEast)
            | (VerticalBound, NorthWest) => {
                let mut add = |x: i8, y: i8| {
                    if let Ok(wall) = self.0.relative_wall(x, y, NorthEast) {
                        walls.push(wall);
                    }
                };
                match self.0.difference(&other.0, NorthEast) {
                    (2, 1, FrontRight) => {
                        add(1, 1);
                    }
                    (2, 0, Right) | (2, -1, BackRight) => {
                        add(1, 1);
                        add(2, 0);
                    }
                    (x, y, Front) if x == y => {
                        for dx in 1..x + 1 {
                            add(dx, dx);
                        }
                    }
                    _ => unreachable!("from: {:?}, to: {:?}", self, other),
                }
            }
            (VerticalBound, NorthEast)
            | (VerticalBound, SouthWest)
            | (HorizontalBound, SouthEast)
            | (HorizontalBound, NorthWest) => {
                let mut add = |x: i8, y: i8| {
                    if let Ok(wall) = self.0.relative_wall(x, y, NorthEast) {
                        walls.push(wall);
                    }
                };
                match self.0.difference(&other.0, NorthEast) {
                    (1, 2, FrontLeft) => {
                        add(1, 1);
                    }
                    (0, 2, Left) | (-1, 2, BackLeft) => {
                        add(1, 1);
                        add(0, 2);
                    }
                    (x, y, Front) if x == y => {
                        for dx in 1..x + 1 {
                            add(dx, dx);
                        }
                    }
                    _ => unreachable!(),
                }
            }
            _ => unreachable!(),
        }
        walls.into()
    }
}

/// An enum that indicates kinds of rotation.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum RotationKind {
    Front,
    Right,
    Left,
    Back,
}

impl<N> RotationNode for RunNode<N> {
    type Kind = RotationKind;
    type Nodes = Vec<(Self, Self::Kind), U4>;

    fn rotation_nodes(&self) -> Self::Nodes {
        use RelativeDirection::*;

        let mut nodes = ForcedVec::new();
        let mut add = |dir, kind| {
            let node = Self(Node {
                x: self.0.x,
                y: self.0.y,
                direction: self.0.direction.rotate(dir),
                _maze_width: PhantomData,
            });
            nodes.push((node, kind));
        };
        add(Front, RotationKind::Front);
        add(Right, RotationKind::Right);
        add(Left, RotationKind::Left);
        add(Back, RotationKind::Back);
        nodes.into()
    }
}

#[cfg(test)]
mod tests {
    use typenum::consts::*;

    use super::*;

    #[test]
    fn test_search_node_new() {
        use AbsoluteDirection::*;

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
            (
                -1,
                0,
                East,
                Err(NodeCreationError {
                    x: -1,
                    y: 0,
                    direction: East,
                }),
            ),
        ];

        for (x, y, dir, expected) in test_cases {
            assert_eq!(SearchNode::<U4>::new(x, y, dir,), expected);
        }
    }

    #[test]
    fn test_search_node_graph() {
        use AbsoluteDirection::*;

        let test_cases = vec![
            (
                (0, 1, North),
                vec![
                    vec![
                        WallNode::Wall((0, 1, false)),
                        WallNode::Node((1, 2, East, Pattern::Search90)),
                    ],
                    vec![
                        WallNode::Wall((0, 1, true)),
                        WallNode::Node((0, 3, North, Pattern::Straight(1))),
                    ],
                    vec![
                        WallNode::Wall((0, 0, true)),
                        WallNode::Node((0, 1, South, Pattern::SpinBack)),
                    ],
                ],
            ),
            (
                (1, 2, East),
                vec![
                    vec![
                        WallNode::Wall((1, 0, true)),
                        WallNode::Node((2, 1, South, Pattern::Search90)),
                    ],
                    vec![
                        WallNode::Wall((1, 1, false)),
                        WallNode::Node((3, 2, East, Pattern::Straight(1))),
                    ],
                    vec![
                        WallNode::Wall((1, 1, true)),
                        WallNode::Node((2, 3, North, Pattern::Search90)),
                    ],
                    vec![
                        WallNode::Wall((0, 1, false)),
                        WallNode::Node((1, 2, West, Pattern::SpinBack)),
                    ],
                ],
            ),
        ];

        type List = Vec<Vec<WallNode<Wall<U4>, (SearchNode<U4>, Pattern)>>>;

        use std::vec::Vec;
        for ((x, y, dir), expected) in test_cases {
            let node = SearchNode::<U4>::new(x, y, dir).unwrap();
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
                                WallNode::Node((SearchNode::<U4>::new(x, y, dir).unwrap(), c))
                            }
                        })
                        .collect()
                })
                .collect();
            assert_eq!(successors, expected, "{:?}", (x, y, dir));
        }
    }

    #[test]
    fn test_run_node_new() {
        use AbsoluteDirection::*;

        let test_cases = vec![
            (
                0,
                0,
                North,
                Ok(RunNode(Node {
                    x: 0,
                    y: 0,
                    direction: North,
                    _maze_width: PhantomData,
                })),
            ),
            (
                0,
                1,
                NorthEast,
                Ok(RunNode(Node {
                    x: 0,
                    y: 1,
                    direction: NorthEast,
                    _maze_width: PhantomData,
                })),
            ),
            (
                0,
                0,
                NorthEast,
                Err(NodeCreationError {
                    x: 0,
                    y: 0,
                    direction: NorthEast,
                }),
            ),
        ];

        for (x, y, dir, expected) in test_cases {
            assert_eq!(RunNode::<U4>::new(x, y, dir,), expected);
        }
    }

    #[test]
    fn test_run_node_graph() {
        use AbsoluteDirection::*;
        use Pattern::*;

        let test_cases = vec![
            (
                (0, 0, North),
                vec![
                    vec![
                        WallNode::Wall((0, 0, true)),
                        WallNode::Wall((0, 1, false)),
                        WallNode::Node((1, 2, NorthEast, FastRun45)),
                        WallNode::Node((2, 2, East, FastRun90)),
                        WallNode::Wall((1, 0, true)),
                        WallNode::Node((2, 1, SouthEast, FastRun135)),
                        WallNode::Node((2, 0, South, FastRun180)),
                    ],
                    vec![WallNode::Wall((0, 0, true))],
                    vec![
                        WallNode::Wall((0, 0, true)),
                        WallNode::Node((0, 2, North, Straight(1))),
                        WallNode::Wall((0, 1, true)),
                    ],
                ],
            ),
            (
                (0, 1, NorthEast),
                vec![
                    vec![
                        WallNode::Wall((0, 1, false)),
                        WallNode::Node((2, 2, East, FastRun45)),
                        WallNode::Wall((1, 0, true)),
                        WallNode::Node((2, 1, SouthEast, FastRunDiagonal90)),
                        WallNode::Node((2, 0, South, FastRun135)),
                    ],
                    vec![
                        WallNode::Wall((0, 1, false)),
                        WallNode::Node((1, 2, NorthEast, StraightDiagonal(1))),
                        WallNode::Wall((1, 1, true)),
                        WallNode::Node((2, 3, NorthEast, StraightDiagonal(2))),
                    ],
                ],
            ),
            (
                (1, 0, NorthEast),
                vec![
                    vec![
                        WallNode::Wall((1, 0, true)),
                        WallNode::Node((2, 2, North, FastRun45)),
                        WallNode::Wall((0, 1, false)),
                        WallNode::Node((1, 2, NorthWest, FastRunDiagonal90)),
                        WallNode::Node((0, 2, West, FastRun135)),
                    ],
                    vec![
                        WallNode::Wall((1, 0, true)),
                        WallNode::Node((2, 1, NorthEast, StraightDiagonal(1))),
                        WallNode::Wall((1, 1, false)),
                        WallNode::Node((3, 2, NorthEast, StraightDiagonal(2))),
                    ],
                ],
            ),
            (
                (2, 0, North),
                vec![
                    vec![
                        WallNode::Wall((1, 0, true)),
                        WallNode::Wall((1, 1, false)),
                        WallNode::Node((3, 2, NorthEast, FastRun45)),
                    ],
                    vec![
                        WallNode::Wall((1, 0, true)),
                        WallNode::Wall((0, 1, false)),
                        WallNode::Node((1, 2, NorthWest, FastRun45)),
                        WallNode::Node((0, 2, West, FastRun90)),
                        WallNode::Wall((0, 0, true)),
                        WallNode::Node((0, 1, SouthWest, FastRun135)),
                        WallNode::Node((0, 0, South, FastRun180)),
                    ],
                    vec![
                        WallNode::Wall((1, 0, true)),
                        WallNode::Node((2, 2, North, Straight(1))),
                        WallNode::Wall((1, 1, true)),
                    ],
                ],
            ),
            (
                (2, 1, NorthWest),
                vec![
                    vec![
                        WallNode::Wall((0, 1, false)),
                        WallNode::Node((0, 2, West, FastRun45)),
                        WallNode::Wall((0, 0, true)),
                        WallNode::Node((0, 1, SouthWest, FastRunDiagonal90)),
                        WallNode::Node((0, 0, South, FastRun135)),
                    ],
                    vec![
                        WallNode::Wall((0, 1, false)),
                        WallNode::Node((1, 2, NorthWest, StraightDiagonal(1))),
                        WallNode::Wall((0, 1, true)),
                        WallNode::Node((0, 3, NorthWest, StraightDiagonal(2))),
                    ],
                ],
            ),
        ];

        use std::vec::Vec;

        type Size = U2;
        type List = Vec<Vec<WallNode<Wall<Size>, (RunNode<Size>, Pattern)>>>;

        for ((x, y, dir), expected) in test_cases {
            let node = RunNode::<Size>::new(x, y, dir).unwrap();
            let successors: List = node
                .successors()
                .into_iter()
                .map(|e| e.into_iter().collect())
                .collect();

            let expected: List = expected
                .into_iter()
                .map(|e| {
                    e.into_iter()
                        .map(|wall_nodes| match wall_nodes {
                            WallNode::Wall((x, y, top)) => {
                                WallNode::Wall(Wall::<Size>::new(x, y, top).unwrap())
                            }
                            WallNode::Node((x, y, dir, c)) => {
                                WallNode::Node((RunNode::<Size>::new(x, y, dir).unwrap(), c))
                            }
                        })
                        .collect()
                })
                .collect();

            assert_eq!(successors, expected, "{:?}", (x, y, dir));
        }
    }

    #[test]
    fn test_run_node_wall_finder() {
        use AbsoluteDirection::*;

        type Size = U4;

        let test_cases = vec![
            (
                (0, 0, North),
                (1, 2, NorthEast),
                vec![(0, 0, true), (0, 1, false)],
            ),
            (
                (0, 1, NorthEast),
                (2, 0, South),
                vec![(0, 1, false), (1, 0, true)],
            ),
            (
                (1, 0, NorthEast),
                (1, 2, NorthWest),
                vec![(1, 0, true), (0, 1, false)],
            ),
            ((2, 1, SouthWest), (1, 0, SouthWest), vec![(0, 0, false)]),
            (
                (2, 0, North),
                (0, 0, South),
                vec![(1, 0, true), (0, 1, false), (0, 0, true)],
            ),
            (
                (0, 0, North),
                (0, 4, North),
                vec![(0, 0, true), (0, 1, true)],
            ),
        ];

        use std::vec::Vec;

        for (src, dst, expected) in test_cases {
            let src = RunNode::<Size>::new(src.0, src.1, src.2).unwrap();
            let dst = RunNode::<Size>::new(dst.0, dst.1, dst.2).unwrap();
            let expected: Vec<Wall<Size>> = expected
                .into_iter()
                .map(|(x, y, top)| Wall::<Size>::new(x, y, top).unwrap())
                .collect();
            let walls: Vec<Wall<Size>> = src.walls_between(&dst).into_iter().collect();
            assert_eq!(walls, expected);
        }
    }

    #[test]
    fn test_search_node_route() {
        use AbsoluteDirection::*;
        use SearchKind::*;

        let test_cases = vec![
            ((0, 1, North), (1, 2, East), Ok(Right)),
            ((0, 1, North), (0, 3, North), Ok(Front)),
            ((0, 1, North), (2, 1, North), Err(())),
            ((2, 1, North), (1, 2, West), Ok(Left)),
            ((0, 1, North), (0, 1, South), Ok(Back)),
        ];

        type Size = U4;

        for (src, dst, expected) in test_cases {
            let src = SearchNode::<Size>::new(src.0, src.1, src.2).unwrap();
            let dst = SearchNode::<Size>::new(dst.0, dst.1, dst.2).unwrap();
            let expected = expected.map_err(|_| RouteError {
                src: src.clone(),
                dst: dst.clone(),
            });
            assert_eq!(src.route(&dst), expected);
        }
    }

    #[test]
    fn test_run_node_route() {
        use AbsoluteDirection::*;
        use RunKind::*;
        use SlalomDirection::{Left as SLeft, Right as SRight};
        use SlalomKind::*;

        let test_cases = vec![
            (
                (0, 0, North),
                (1, 2, NorthEast),
                Ok(Slalom(FastRun45, SRight)),
            ),
            ((0, 0, North), (2, 0, South), Ok(Slalom(FastRun180, SRight))),
            ((0, 0, North), (3, 0, SouthEast), Err(())),
            (
                (1, 0, NorthEast),
                (0, 2, West),
                Ok(Slalom(FastRun135Rev, SLeft)),
            ),
            ((0, 0, North), (0, 6, North), Ok(Straight(3))),
            (
                (0, 1, NorthEast),
                (2, 3, NorthEast),
                Ok(StraightDiagonal(2)),
            ),
            (
                (2, 1, NorthWest),
                (0, 1, SouthWest),
                Ok(Slalom(FastRunDiagonal90, SLeft)),
            ),
            (
                (2, 0, East),
                (3, 2, NorthWest),
                Ok(Slalom(FastRun135, SLeft)),
            ),
            (
                (1, 2, SouthWest),
                (0, 0, South),
                Ok(Slalom(FastRun45Rev, SLeft)),
            ),
        ];

        type Size = U4;

        for (src, dst, expected) in test_cases {
            let src = RunNode::<Size>::new(src.0, src.1, src.2).unwrap();
            let dst = RunNode::<Size>::new(dst.0, dst.1, dst.2).unwrap();
            let expected = expected.map_err(|_| RouteError {
                src: src.clone(),
                dst: dst.clone(),
            });
            assert_eq!(src.route(&dst), expected);
        }
    }

    #[test]
    fn test_search_node_into_wall() {
        use AbsoluteDirection::*;

        type Size = U4;

        let test_cases = vec![
            ((1, 0, East), (0, 0, false)),
            ((0, 1, North), (0, 0, true)),
            ((1, 0, West), (0, 0, false)),
            ((2, 1, South), (1, 0, true)),
            ((3, 2, East), (1, 1, false)),
        ];

        for (node, expected) in test_cases {
            let node = SearchNode::<Size>::new(node.0, node.1, node.2).unwrap();
            let expected = Wall::<Size>::new(expected.0, expected.1, expected.2).unwrap();
            assert_eq!(Wall::from(node), expected);
        }
    }

    #[test]
    fn test_run_node_rotation() {
        use AbsoluteDirection::*;
        use RotationKind::*;

        type Size = U4;

        let test_cases = vec![
            (
                (0, 0, East),
                vec![
                    ((0, 0, East), Front),
                    ((0, 0, South), Right),
                    ((0, 0, North), Left),
                    ((0, 0, West), Back),
                ],
            ),
            (
                (2, 0, South),
                vec![
                    ((2, 0, South), Front),
                    ((2, 0, West), Right),
                    ((2, 0, East), Left),
                    ((2, 0, North), Back),
                ],
            ),
        ];

        use std::vec::Vec;

        for (node, expected) in test_cases {
            let node = RunNode::<Size>::new(node.0, node.1, node.2).unwrap();
            let expected = expected
                .into_iter()
                .map(|(node, kind)| (RunNode::<Size>::new(node.0, node.1, node.2).unwrap(), kind))
                .collect::<Vec<_>>();
            assert_eq!(node.rotation_nodes(), expected.as_slice());
        }
    }
}
