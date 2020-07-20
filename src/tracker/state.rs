use core::ops::Div;

use quantities::{Angle, Distance, Time, TimeDifferentiable};

use crate::{ddt, dt};

pub struct State {
    pub x: SubState<Distance>,
    pub y: SubState<Distance>,
    pub theta: SubState<Angle>,
}

pub struct SubState<T>
where
    T: TimeDifferentiable,
    dt!(T): TimeDifferentiable,
{
    pub x: T,
    pub v: dt!(T),
    pub a: ddt!(T),
}
