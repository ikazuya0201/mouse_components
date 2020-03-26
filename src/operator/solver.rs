use heapless::{ArrayLength, Vec};

use super::maze::{Graph, Node};

pub trait Solver<N, G>
where
    N: Node,
    G: Graph<N>,
{
    fn solve<L: ArrayLength<N>>(&self, current: N, graph: &G) -> Vec<N, L>;
}
