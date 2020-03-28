use heapless::{ArrayLength, Vec};

use super::maze::Graph;

pub trait Solver<Node, Cost, Direction, G>
where
    G: Graph<Node, Cost, Direction>,
{
    //return: (route, last direction mapping)
    fn solve<L: ArrayLength<Node>>(
        &self,
        current: Node,
        graph: &G,
    ) -> (Vec<Node, L>, fn(fn(Direction) -> bool) -> Direction);
}
