use heapless::{ArrayLength, Vec};

use super::maze::{Cost, Graph, Node};

pub trait Solver<N, C, G>
where
    N: Node,
    C: Cost,
    G: Graph<N, C>,
{
    fn solve<L: ArrayLength<N>>(&self, current: N, graph: &G) -> Vec<N, L>;
}
