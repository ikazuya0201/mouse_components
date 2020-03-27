use heapless::{consts::*, ArrayLength, Vec};

use super::direction::AbsoluteDirection;
use super::maze::{Cost, Graph, Node};

pub trait Solver<N, C, D, G>
where
    N: Node,
    C: Cost,
    D: AbsoluteDirection,
    G: Graph<N, C, D>,
{
    //return: (route, last directions)
    fn solve<L: ArrayLength<N>>(&self, current: N, graph: &G) -> (Vec<N, L>, Vec<D, U4>);
}
