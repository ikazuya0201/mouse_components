mod node;

use core::ops::Mul;

use generic_array::{ArrayLength, GenericArray};
use heapless::{consts::*, Vec};
use static_assertions::const_assert;
use typenum::{PowerOfTwo, Unsigned};

use crate::direction::AbsoluteDirection;
use crate::operator::{
    CheckableGraph, DirectionInstructor, DirectionalGraph, Graph, GraphTranslator,
};
use node::{Location, Node};

pub struct Maze<H, W, T, U>
where
    H: Mul<W>,
    <H as Mul<W>>::Output: Mul<U2>,
    <<H as Mul<W>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
    T: IntoIterator<Item = (Node<H, W>, u16, U)>,
    U: IntoIterator<Item = Node<H, W>>,
{
    is_checked: GenericArray<bool, <<H as Mul<W>>::Output as Mul<U2>>::Output>,
    is_wall: GenericArray<bool, <<H as Mul<W>>::Output as Mul<U2>>::Output>,
    tables: [T; 3],
}

impl<H, W, T, U> Maze<H, W, T, U>
where
    H: Mul<W> + Unsigned + PowerOfTwo,
    W: Unsigned + PowerOfTwo,
    <H as Mul<W>>::Output: Mul<U2>,
    <<H as Mul<W>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
    T: IntoIterator<Item = (Node<H, W>, u16, U)>,
    U: IntoIterator<Item = Node<H, W>>,
{
    pub fn new(tables: [T; 3]) -> Self {
        Self {
            is_checked: GenericArray::default(),
            is_wall: GenericArray::default(),
            tables,
        }
    }

    #[inline]
    fn y_offset() -> u32 {
        W::USIZE.trailing_zeros()
    }

    #[inline]
    fn z_offset() -> u32 {
        W::USIZE.trailing_zeros() + H::USIZE.trailing_zeros()
    }

    fn is_wall(&self, x: usize, y: usize, z: bool) -> bool {
        if z {
            self.is_wall[x | (y << Self::y_offset()) | (1 << Self::z_offset())]
        } else {
            self.is_wall[x | (y << Self::y_offset())]
        }
    }

    fn wall_exists(&self, node: Node<H, W>) -> bool {
        use Location::*;
        match node.location() {
            Cell => false,
            HorizontalBound => {
                let x = node.x() as usize / 2;
                let y = node.y() as usize / 2;
                self.is_wall(x, y, true)
            }
            VerticalBound => {
                let x = node.x() as usize / 2;
                let y = node.y() as usize / 2;
                self.is_wall(x, y, false)
            }
        }
    }

    fn next_straight(&self, node: Node<H, W>) -> Option<Node<H, W>> {
        let next = node.next_straight()?;
        if self.wall_exists(next) {
            None
        } else {
            Some(next)
        }
    }
}

impl<H, W, T, U> Graph<Node<H, W>, u16> for Maze<H, W, T, U>
where
    H: Mul<W>,
    <H as Mul<W>>::Output: Mul<U2>,
    <<H as Mul<W>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
    T: IntoIterator<Item = (Node<H, W>, u16, U)>,
    U: IntoIterator<Item = Node<H, W>>,
{
    type Edges = Vec<(Node<H, W>, u16), U4096>;

    fn successors(&self, node: Node<H, W>) -> Self::Edges {
        let mut succs = Vec::new();
        succs
    }

    fn predecessors(&self, node: Node<H, W>) -> Self::Edges {
        let mut preds = Vec::new();
        preds
    }
}
