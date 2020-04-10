use heapless::{ArrayLength, Vec};

use super::maze::{CheckableGraph, DirectionalGraph};

pub trait Solver<Node, Cost, Direction, G, L>
where
    G: DirectionalGraph<Node, Cost, Direction> + CheckableGraph<Node, Cost, L>,
    L: ArrayLength<Node>,
{
    //return: (route, last direction mapping)
    fn start(&self) -> Node;
    fn solve(
        &self,
        current: Node,
        graph: &G,
    ) -> Option<(Vec<Node, L>, fn(fn(Direction) -> bool) -> Direction)>;
}
