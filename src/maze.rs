mod node;

use core::ops::Mul;

use generic_array::{ArrayLength, GenericArray};
use heapless::{consts::*, Vec};
use static_assertions::const_assert;
use typenum::{PowerOfTwo, Unsigned};

use crate::direction::AbsoluteDirection;
use crate::operator::{
    CheckableGraph, DirectionalGraph, Graph, GraphTranslator, PatternInstructor,
};
use node::{Location, Node, NodeId};

pub struct Maze<H, W>
where
    H: Mul<W>,
    <H as Mul<W>>::Output: Mul<U2>,
    <<H as Mul<W>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
{
    is_checked: GenericArray<bool, <<H as Mul<W>>::Output as Mul<U2>>::Output>,
    is_wall: GenericArray<bool, <<H as Mul<W>>::Output as Mul<U2>>::Output>,
}

impl<H, W> Maze<H, W>
where
    H: Mul<W> + Unsigned + PowerOfTwo,
    W: Unsigned + PowerOfTwo,
    <H as Mul<W>>::Output: Mul<U2>,
    <<H as Mul<W>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
{
    pub fn new() -> Self {
        Self {
            is_checked: GenericArray::default(),
            is_wall: GenericArray::default(),
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

    fn wall_exists(&self, node: NodeId<H, W>) -> bool {
        use Location::*;
        let node = node.as_node();
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
}

impl<H, W> Graph<Node<H, W>, u16> for Maze<H, W>
where
    H: Mul<W>,
    <H as Mul<W>>::Output: Mul<U2>,
    <<H as Mul<W>>::Output as Mul<U2>>::Output: ArrayLength<bool>,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_successors() {}
}
