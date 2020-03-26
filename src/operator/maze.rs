use heapless::{ArrayLength, Vec};

use super::agent::Position;

pub trait Node: PartialEq + Eq + PartialOrd + Ord {}

pub trait Cost: PartialEq + Eq + PartialOrd + Ord {}

///This is an interface of nodes and points.
pub trait NodePosition<N: Node, P: Position> {
    fn node_to_position(&self, node: N) -> P;
    fn position_to_node(&self, position: P) -> N;
}

//Maze implementation should implement Maze and NodePosition
pub trait Graph<N: Node, C: Cost> {
    fn neighbors<L: ArrayLength<N>>(&self, node: N) -> Vec<(N, C), L>;
}
