use core::ops::Div;

use quantities::{Angle, Distance, Quantity, Time, TimeDifferentiable};

use crate::{ddt, dt};

#[derive(Clone, Debug)]
pub struct State {
    pub x: SubState<Distance>,
    pub y: SubState<Distance>,
    pub theta: SubState<Angle>,
}

#[derive(Clone, Debug)]
pub struct SubState<T>
where
    T: TimeDifferentiable + core::fmt::Debug,
    dt!(T): TimeDifferentiable + core::fmt::Debug,
    ddt!(T): Quantity + core::fmt::Debug,
{
    pub x: T,
    pub v: <T as Div<Time>>::Output,
    pub a: <<T as Div<Time>>::Output as Div<Time>>::Output,
}
