use core::ops::Div;

use quantities::{Angle, Distance, Quantity, Time, TimeDifferentiable};

use crate::{dddt, ddt, dt};

#[derive(Clone, Copy, Debug, Default)]
pub struct SubTarget<T>
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

#[derive(Clone, Copy, Debug)]
pub struct Target {
    pub x: SubTarget<Distance>,
    pub y: SubTarget<Distance>,
    pub theta: SubTarget<Angle>,
}
