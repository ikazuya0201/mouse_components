use core::marker::PhantomData;

use typenum::{PowerOfTwo, Unsigned};

use super::direction::{AbsoluteDirection, RelativeDirection};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Location {
    Cell,
    VerticalBound,
    HorizontalBound,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, PartialOrd, Ord)]
pub struct Position<N> {
    x: i16,
    y: i16,
    _size: PhantomData<fn() -> N>,
}

impl<N> Position<N>
where
    N: Unsigned,
{
    pub fn new(x: i16, y: i16) -> Self {
        Self {
            x,
            y,
            _size: PhantomData,
        }
    }

    pub fn relative_node(&self, dx: i16, dy: i16, direction: AbsoluteDirection) -> Node<N> {
        Node::new(self.x + dx, self.y + dy, direction)
    }

    pub fn relative_position(&self, dx: i16, dy: i16) -> Self {
        Self::new(self.x + dx, self.y + dy)
    }

    pub fn difference(&self, to: &Self) -> (i16, i16) {
        (to.x - self.x, to.y - self.y)
    }

    #[inline]
    pub fn x(&self) -> i16 {
        self.x
    }

    #[inline]
    pub fn y(&self) -> i16 {
        self.y
    }

    #[inline]
    fn x_is_even(&self) -> bool {
        self.x() & 1 == 0
    }

    #[inline]
    fn y_is_even(&self) -> bool {
        self.y() & 1 == 0
    }

    pub fn location(&self) -> Location {
        use Location::*;
        if self.x_is_even() {
            if self.y_is_even() {
                Cell
            } else {
                HorizontalBound
            }
        } else {
            if self.y_is_even() {
                VerticalBound
            } else {
                unreachable!()
            }
        }
    }
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub struct RelativePositionError<N> {
    from: Node<N>,
    x_diff: i16,
    y_diff: i16,
    base_dir: AbsoluteDirection,
}

#[derive(Clone, PartialEq, Eq, Debug, PartialOrd, Ord)]
pub struct Node<N> {
    position: Position<N>,
    direction: AbsoluteDirection,
}

impl<N> Node<N>
where
    N: Unsigned,
{
    pub fn new(x: i16, y: i16, direction: AbsoluteDirection) -> Self {
        Self {
            position: Position::new(x, y),
            direction,
        }
    }

    #[inline]
    pub fn position(&self) -> Position<N> {
        self.position
    }

    #[inline]
    pub fn x(&self) -> i16 {
        self.position().x()
    }

    #[inline]
    pub fn y(&self) -> i16 {
        self.position().y()
    }

    #[inline]
    pub fn direction(&self) -> AbsoluteDirection {
        self.direction
    }

    pub fn get_relative_node(
        &self,
        x_diff: i16,
        y_diff: i16,
        dir_diff: RelativeDirection,
        base_dir: AbsoluteDirection,
    ) -> Result<Self, RelativePositionError<N>> {
        let position = self.get_relative_position(x_diff, y_diff, base_dir)?;
        let direction = self.direction.rotate(dir_diff);
        Ok(Node::<N>::new(position.x(), position.y(), direction))
    }

    pub fn get_relative_position(
        &self,
        x_diff: i16,
        y_diff: i16,
        base_dir: AbsoluteDirection,
    ) -> Result<Position<N>, RelativePositionError<N>> {
        use RelativeDirection::*;
        let relative_direction = base_dir.relative(self.direction);
        match relative_direction {
            Front => Ok(Position::new(self.x() + x_diff, self.y() + y_diff)),
            Right => Ok(Position::new(self.x() + y_diff, self.y() - x_diff)),
            Back => Ok(Position::new(self.x() - x_diff, self.y() - y_diff)),
            Left => Ok(Position::new(self.x() - y_diff, self.y() + x_diff)),
            _ => Err(RelativePositionError {
                from: self.clone(),
                x_diff,
                y_diff,
                base_dir,
            }),
        }
    }

    pub fn difference(
        &self,
        to: &Self,
        base_dir: AbsoluteDirection,
    ) -> (i16, i16, RelativeDirection) {
        use RelativeDirection::*;

        let (dx, dy) = self.position.difference(&to.position);
        let (dx, dy) = match base_dir.relative(self.direction) {
            Front => (dx, dy),
            Right => (-dy, dx),
            Back => (-dx, -dy),
            Left => (dy, -dx),
            _ => unreachable!(),
        };
        let relative_direction = self.direction.relative(to.direction);
        (dx, dy, relative_direction)
    }

    pub fn location(&self) -> Location {
        self.position.location()
    }
}

impl<N> Node<N>
where
    N: Unsigned + PowerOfTwo,
{
    pub fn to_node_id(&self) -> Result<NodeId<N>, NodeCreationError> {
        let x = self.x();
        let y = self.y();
        let direction = self.direction();

        if x < 0 || y < 0 {
            Err(NodeCreationError { x, y, direction })
        } else {
            NodeId::new(x as u16, y as u16, direction)
        }
    }

    pub fn to_search_node_id(&self) -> Result<SearchNodeId<N>, NodeCreationError> {
        let x = self.x();
        let y = self.y();
        let direction = self.direction();

        if x < 0 || y < 0 {
            Err(NodeCreationError { x, y, direction })
        } else {
            SearchNodeId::new(x as u16, y as u16, direction)
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct NodeId<N> {
    raw: u16,
    _size: PhantomData<fn() -> N>,
}

impl<N> NodeId<N>
where
    N: Unsigned,
{
    #[inline]
    fn x_max() -> u16 {
        N::U16 * 2 - 1
    }

    #[inline]
    fn y_max() -> u16 {
        N::U16 * 2 - 1
    }
}

impl<N> NodeId<N>
where
    N: Unsigned + PowerOfTwo,
{
    pub fn new(x: u16, y: u16, direction: AbsoluteDirection) -> Result<Self, NodeCreationError> {
        use AbsoluteDirection::*;

        let create_error = || {
            Err(NodeCreationError {
                x: x as i16,
                y: y as i16,
                direction,
            })
        };

        if x > Self::x_max() || y > Self::y_max() {
            return create_error();
        }
        let direction = if x & 1 == 0 {
            if y & 1 == 0 {
                match direction {
                    North => 0,
                    East => 1,
                    South => 2,
                    West => 3,
                    _ => return create_error(),
                }
            } else {
                match direction {
                    NorthEast => 0,
                    SouthEast => 1,
                    SouthWest => 2,
                    NorthWest => 3,
                    _ => return create_error(),
                }
            }
        } else {
            if y & 1 == 0 {
                match direction {
                    NorthEast => 0,
                    SouthEast => 1,
                    SouthWest => 2,
                    NorthWest => 3,
                    _ => return create_error(),
                }
            } else {
                return create_error();
            }
        };
        Ok(Self {
            raw: x | (y << Self::y_offset()) | (direction << Self::direction_offset()),
            _size: PhantomData,
        })
    }

    pub fn to_node(&self) -> Node<N> {
        use AbsoluteDirection::*;
        let x = self.x_raw();
        let y = self.y_raw();
        let direction = if x & 1 == 0 {
            if y & 1 == 0 {
                match self.raw >> Self::direction_offset() {
                    0 => North,
                    1 => East,
                    2 => South,
                    3 => West,
                    _ => unreachable!(),
                }
            } else {
                match self.raw >> Self::direction_offset() {
                    0 => NorthEast,
                    1 => SouthEast,
                    2 => SouthWest,
                    3 => NorthWest,
                    _ => unreachable!(),
                }
            }
        } else {
            if y & 1 == 0 {
                match self.raw >> Self::direction_offset() {
                    0 => NorthEast,
                    1 => SouthEast,
                    2 => SouthWest,
                    3 => NorthWest,
                    _ => unreachable!(),
                }
            } else {
                unreachable!()
            }
        };
        Node::<N>::new(x as i16, y as i16, direction)
    }

    #[inline]
    fn y_offset() -> u32 {
        (N::USIZE * 2).trailing_zeros()
    }

    #[inline]
    fn direction_offset() -> u32 {
        2 * (N::USIZE * 2).trailing_zeros()
    }

    #[inline]
    fn x_raw(&self) -> u16 {
        self.raw & Self::x_max()
    }

    #[inline]
    fn y_raw(&self) -> u16 {
        (self.raw >> Self::y_offset()) & Self::y_max()
    }
}

impl<N> Into<Node<N>> for NodeId<N>
where
    N: Unsigned + PowerOfTwo,
{
    fn into(self) -> Node<N> {
        self.to_node()
    }
}

impl<N> Into<usize> for NodeId<N> {
    fn into(self) -> usize {
        self.raw.into()
    }
}

impl<N> core::convert::TryFrom<Node<N>> for NodeId<N>
where
    N: Unsigned + PowerOfTwo,
{
    type Error = NodeCreationError;

    fn try_from(value: Node<N>) -> Result<NodeId<N>, Self::Error> {
        value.to_node_id()
    }
}

impl<N> core::fmt::Debug for NodeId<N>
where
    N: Unsigned + PowerOfTwo,
    Node<N>: core::fmt::Debug,
{
    fn fmt(&self, fmt: &mut core::fmt::Formatter) -> core::fmt::Result {
        let node: Node<N> = <NodeId<N> as Into<Node<N>>>::into(*self);
        writeln!(fmt, "{:?}", node)
    }
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct SearchNodeId<N> {
    raw: u16,
    _size: PhantomData<fn() -> N>,
}

impl<N> SearchNodeId<N>
where
    N: Unsigned,
{
    #[inline]
    fn x_max() -> u16 {
        N::U16 * 2 - 1
    }

    #[inline]
    fn y_max() -> u16 {
        N::U16 * 2 - 1
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct NodeCreationError {
    x: i16,
    y: i16,
    direction: AbsoluteDirection,
}

impl<N> SearchNodeId<N>
where
    N: Unsigned + PowerOfTwo,
{
    pub fn new(x: u16, y: u16, direction: AbsoluteDirection) -> Result<Self, NodeCreationError> {
        use AbsoluteDirection::*;
        let create_error = || {
            Err(NodeCreationError {
                x: x as i16,
                y: y as i16,
                direction,
            })
        };

        if x > Self::x_max() || y > Self::y_max() {
            return create_error();
        }

        let direction = if x & 1 == 1 {
            if y & 1 == 1 {
                return create_error();
            } else {
                match direction {
                    East => 0,
                    West => 1,
                    _ => return create_error(),
                }
            }
        } else {
            if y & 1 == 1 {
                match direction {
                    North => 0,
                    South => 1,
                    _ => return create_error(),
                }
            } else {
                return create_error();
            }
        };

        Ok(Self {
            raw: x | (y << Self::y_offset()) | (direction << Self::direction_offset()),
            _size: PhantomData,
        })
    }

    pub fn to_node(&self) -> Node<N> {
        use AbsoluteDirection::*;
        let x = self.x_raw();
        let y = self.y_raw();
        let direction = if x & 1 == 1 {
            if y & 1 == 1 {
                unreachable!()
            } else {
                match self.raw >> Self::direction_offset() {
                    0 => East,
                    1 => West,
                    _ => unreachable!(),
                }
            }
        } else {
            if y & 1 == 1 {
                match self.raw >> Self::direction_offset() {
                    0 => North,
                    1 => South,
                    _ => unreachable!(),
                }
            } else {
                unreachable!()
            }
        };
        Node::<N>::new(x as i16, y as i16, direction)
    }

    #[inline]
    fn y_offset() -> u32 {
        (N::USIZE * 2).trailing_zeros()
    }

    #[inline]
    fn direction_offset() -> u32 {
        2 * (N::USIZE * 2).trailing_zeros()
    }

    #[inline]
    fn x_raw(&self) -> u16 {
        self.raw & Self::x_max()
    }

    #[inline]
    fn y_raw(&self) -> u16 {
        (self.raw >> Self::y_offset()) & Self::y_max()
    }
}

impl<N> Into<Node<N>> for SearchNodeId<N>
where
    N: Unsigned + PowerOfTwo,
{
    fn into(self) -> Node<N> {
        self.to_node()
    }
}

impl<N> Into<usize> for SearchNodeId<N> {
    fn into(self) -> usize {
        self.raw.into()
    }
}

impl<N> core::convert::TryFrom<Node<N>> for SearchNodeId<N>
where
    N: Unsigned + PowerOfTwo,
{
    type Error = NodeCreationError;

    fn try_from(value: Node<N>) -> Result<SearchNodeId<N>, Self::Error> {
        value.to_search_node_id()
    }
}

impl<N> core::fmt::Debug for SearchNodeId<N>
where
    N: Unsigned + PowerOfTwo,
    Node<N>: core::fmt::Debug,
{
    fn fmt(&self, fmt: &mut core::fmt::Formatter) -> core::fmt::Result {
        let node: Node<N> = <SearchNodeId<N> as Into<Node<N>>>::into(*self);
        writeln!(fmt, "{:?}", node)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use typenum::consts::*;
    use AbsoluteDirection::*;

    #[test]
    fn test_base_methods() {
        let test_data = vec![
            ((0u16, 0u16, North), true),
            ((0, 0, East), true),
            ((0, 0, South), true),
            ((0, 0, West), true),
            ((0, 1, NorthEast), true),
            ((0, 1, SouthEast), true),
            ((0, 1, SouthWest), true),
            ((0, 1, NorthWest), true),
            ((1, 0, NorthEast), true),
            ((1, 0, SouthEast), true),
            ((1, 0, SouthWest), true),
            ((1, 0, NorthWest), true),
            ((4, 4, South), true),
            ((5, 4, NorthEast), true),
            ((4, 5, NorthWest), true),
            ((14, 14, North), true),
            ((15, 14, SouthEast), true),
            ((14, 15, SouthWest), true),
            ((30, 30, North), true),
            ((31, 30, NorthEast), true),
            ((30, 31, SouthEast), true),
            ((32, 31, NorthEast), false),
            ((31, 32, NorthEast), false),
            ((0, 0, NorthEast), false),
            ((0, 1, East), false),
            ((1, 0, North), false),
        ];

        for ((x, y, direction), is_ok) in test_data {
            let node = NodeId::<U16>::new(x, y, direction);
            assert_eq!(node.is_ok(), is_ok);
            if !is_ok {
                continue;
            }
            let node = node.unwrap().to_node();
            assert_eq!(node.x() as u16, x);
            assert_eq!(node.y() as u16, y);
            assert_eq!(node.direction(), direction);
        }
    }

    #[test]
    fn test_to_node_id() {
        let test_data = vec![
            (-1, 0, West, false),
            (0, 100, North, false),
            (0, 0, North, true),
            (31, 30, SouthEast, true),
        ];

        for (x, y, direction, is_ok) in test_data {
            let expected = if is_ok {
                NodeId::<U16>::new(x as u16, y as u16, direction)
            } else {
                Err(NodeCreationError { x, y, direction })
            };
            let node = Node::<U16>::new(x, y, direction);
            assert_eq!(node.to_node_id(), expected);
        }
    }

    #[test]
    fn test_to_search_node_id() {
        let test_data = vec![
            (-1, 0, West, false),
            (0, 100, North, false),
            (0, 0, North, false),
            (31, 30, SouthEast, false),
            (0, 1, North, true),
            (0, 1, South, true),
            (1, 0, East, true),
            (1, 0, West, true),
            (30, 31, South, true),
            (31, 30, East, true),
            (1, 0, North, false),
            (1, 0, NorthEast, false),
            (0, 1, East, false),
            (0, 1, SouthEast, false),
        ];

        for (x, y, direction, is_ok) in test_data {
            let expected = if is_ok {
                SearchNodeId::<U16>::new(x as u16, y as u16, direction)
            } else {
                Err(NodeCreationError { x, y, direction })
            };
            let node = Node::<U16>::new(x, y, direction);
            assert_eq!(node.to_search_node_id(), expected);
        }
    }
}
