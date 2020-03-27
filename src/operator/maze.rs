use heapless::{ArrayLength, Vec};

use super::agent::Position;
use super::direction::AbsoluteDirection;

pub trait Node: PartialEq + Eq + PartialOrd + Ord {}

pub trait Cost: PartialEq + Eq + PartialOrd + Ord {}

pub trait GraphTranslator<N: Node, P: Position> {
    fn node_to_position(&self, node: N) -> P;
    fn position_to_node(&self, position: P) -> N;
}

pub trait Graph<N: Node, C: Cost, D: AbsoluteDirection> {
    fn neighbors<L: ArrayLength<(N, C)>>(&self, node: N) -> Vec<(N, C), L>;
    fn is_edge_checked(&self, node1: N, node2: N) -> bool;
    fn edge_direction(&self, src: N, dst: N) -> D;
}
