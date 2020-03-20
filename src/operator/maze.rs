use heapless::{ArrayLength, Vec};

use super::agent::Position;

pub trait Node {}

///This is an interface of nodes and points.
pub trait NodePosition<N: Node, P: Position> {
    fn node_to_position(&self, node: N) -> P;
    fn position_to_node(&self, position: P) -> N;
}

//Maze implementation should implement Maze and NodePosition
pub trait Maze<N: Node> {
    fn neighbors<L: ArrayLength<N>>(&self, node: N) -> Vec<N, L>;
}
