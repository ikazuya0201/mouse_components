mod node;

use heapless::{consts::*, Vec};
use static_assertions::const_assert;

use crate::direction::AbsoluteDirection;
use crate::operator::{
    CheckableGraph, DirectionInstructor, DirectionalGraph, Graph, GraphTranslator,
};
use node::{Location, Node};

const MAX_H: usize = 32;
const MAX_W: usize = 32;

const_assert!(MAX_H.is_power_of_two());
const_assert!(MAX_W.is_power_of_two());

pub struct Maze {
    h: usize,
    w: usize,
    is_checked: [bool; MAX_H * MAX_W * 2],
    is_wall: [bool; MAX_H * MAX_W * 2],
}

impl Maze {
    pub fn new(h: usize, w: usize) -> Self {
        Self {
            h,
            w,
            is_checked: [false; MAX_H * MAX_W * 2],
            is_wall: [false; MAX_H * MAX_W * 2],
        }
    }

    const fn y_offset() -> u32 {
        MAX_W.trailing_zeros()
    }

    const fn z_offset() -> u32 {
        MAX_W.trailing_zeros() + MAX_H.trailing_zeros()
    }

    fn is_wall(&self, x: usize, y: usize, z: bool) -> bool {
        if z {
            self.is_wall[x | (y << Self::y_offset()) | (1 << Self::z_offset())]
        } else {
            self.is_wall[x | (y << Self::y_offset())]
        }
    }

    fn wall_exists(&self, node: Node) -> bool {
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

    fn next_straight(&self, node: Node) -> Option<Node> {
        let next = node.next_straight()?;
        if self.wall_exists(next) {
            None
        } else {
            Some(next)
        }
    }
}

impl Graph<Node, u16> for Maze {
    type Edges = Vec<(Node, u16), U4096>;

    fn successors(&self, node: Node) -> Self::Edges {
        let mut succs = Vec::new();
        succs
    }

    fn predecessors(&self, node: Node) -> Self::Edges {
        let mut preds = Vec::new();
        preds
    }
}
