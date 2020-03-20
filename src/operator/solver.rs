use heapless::{ArrayLength, Vec};

use super::maze::{Maze, Node};

pub trait Solver<N>
where
    N: Node,
{
    fn solve<M: Maze<N>, L: ArrayLength<N>>(start: &[N], goals: &[N], maze: &M) -> Vec<N, L>;
}
