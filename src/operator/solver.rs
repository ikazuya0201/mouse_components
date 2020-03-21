use heapless::{ArrayLength, Vec};

use super::maze::{Graph, Node};

pub trait Solver<N>
where
    N: Node,
{
    fn solve<M: Graph<N>, L: ArrayLength<N>>(start: &[N], goals: &[N], maze: &M) -> Vec<N, L>;
}
