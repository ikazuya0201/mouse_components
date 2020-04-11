use heapless::{ArrayLength, Vec};

use super::maze::{CheckableGraph, DirectionalGraph};

pub trait Solver<Node, Cost, Direction, G, L>
where
    G: DirectionalGraph<Node, Cost, Direction> + CheckableGraph<Node, Cost>,
    L: ArrayLength<Node>,
{
    type Directions: IntoIterator<Item = Direction>;
    //return: (route, last direction candidates sequence)
    fn start(&self) -> Node;
    fn solve(&self, current: Node, graph: &G) -> Option<(Vec<Node, L>, Self::Directions)>;
}
