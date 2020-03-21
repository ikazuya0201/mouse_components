use heapless::{ArrayLength, Vec};

use super::agent::Position;

pub trait Target {}

pub trait TrajectoryGenerator<P, T>
where
    T: Target,
    P: Position,
{
    fn generate<L: ArrayLength<T>>(&self, positions: &[P]) -> Vec<T, L>;
}
