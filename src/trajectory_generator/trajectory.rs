use core::ops::Div;

use quantities::{Quantity, Time, TimeDifferentiable};

#[derive(Clone, Debug)]
pub struct Target<T>
where
    T: TimeDifferentiable,
    dt!(T): TimeDifferentiable,
    ddt!(T): TimeDifferentiable,
    dddt!(T): Quantity,
{
    pub x: T,
    pub v: <T as Div<Time>>::Output,
    pub a: <<T as Div<Time>>::Output as Div<Time>>::Output,
    pub j: <<<T as Div<Time>>::Output as Div<Time>>::Output as Div<Time>>::Output,
}
