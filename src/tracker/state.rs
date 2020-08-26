use core::ops::Div;

use quantities::{Angle, Distance, Quantity, Time, TimeDifferentiable};

use crate::{ddt, dt};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct State {
    pub x: SubState<Distance>,
    pub y: SubState<Distance>,
    pub theta: SubState<Angle>,
}

#[derive(Clone, Debug, Default, PartialEq)]
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

#[cfg(test)]
mod tests {
    use super::*;
    use approx::{AbsDiffEq, RelativeEq};

    const V_RATIO: f32 = 100.0;
    const A_RATIO: f32 = 10000.0;

    impl AbsDiffEq for SubState<Distance> {
        type Epsilon = f32;

        fn default_epsilon() -> Self::Epsilon {
            std::f32::EPSILON
        }

        fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
            self.x
                .as_meters()
                .abs_diff_eq(&other.x.as_meters(), epsilon)
                && self
                    .v
                    .as_meter_per_second()
                    .abs_diff_eq(&other.v.as_meter_per_second(), epsilon * V_RATIO)
                && self
                    .a
                    .as_meter_per_second_squared()
                    .abs_diff_eq(&other.a.as_meter_per_second_squared(), epsilon * A_RATIO)
        }
    }

    impl AbsDiffEq for SubState<Angle> {
        type Epsilon = f32;

        fn default_epsilon() -> Self::Epsilon {
            std::f32::EPSILON
        }

        fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
            self.x
                .as_radian()
                .abs_diff_eq(&other.x.as_radian(), epsilon)
                && self
                    .v
                    .as_radian_per_second()
                    .abs_diff_eq(&other.v.as_radian_per_second(), epsilon * V_RATIO)
                && self
                    .a
                    .as_radian_per_second_squared()
                    .abs_diff_eq(&other.a.as_radian_per_second_squared(), epsilon * A_RATIO)
        }
    }

    impl RelativeEq for SubState<Distance> {
        fn default_max_relative() -> Self::Epsilon {
            Self::default_epsilon()
        }

        fn relative_eq(
            &self,
            other: &Self,
            epsilon: Self::Epsilon,
            max_relative: Self::Epsilon,
        ) -> bool {
            self.x
                .as_meters()
                .relative_eq(&other.x.as_meters(), epsilon, max_relative)
                && self.v.as_meter_per_second().relative_eq(
                    &other.v.as_meter_per_second(),
                    epsilon * V_RATIO,
                    max_relative,
                )
                && self.a.as_meter_per_second_squared().relative_eq(
                    &other.a.as_meter_per_second_squared(),
                    epsilon * A_RATIO,
                    max_relative,
                )
        }
    }

    impl RelativeEq for SubState<Angle> {
        fn default_max_relative() -> Self::Epsilon {
            Self::default_epsilon()
        }

        fn relative_eq(
            &self,
            other: &Self,
            epsilon: Self::Epsilon,
            max_relative: Self::Epsilon,
        ) -> bool {
            self.x
                .as_radian()
                .relative_eq(&other.x.as_radian(), epsilon, max_relative)
                && self.v.as_radian_per_second().relative_eq(
                    &other.v.as_radian_per_second(),
                    epsilon * V_RATIO,
                    max_relative,
                )
                && self.a.as_radian_per_second_squared().relative_eq(
                    &other.a.as_radian_per_second_squared(),
                    epsilon * A_RATIO,
                    max_relative,
                )
        }
    }

    impl AbsDiffEq for State {
        type Epsilon = f32;

        fn default_epsilon() -> Self::Epsilon {
            std::f32::EPSILON
        }

        fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
            self.x.abs_diff_eq(&other.x, epsilon)
                && self.y.abs_diff_eq(&other.y, epsilon)
                && self.theta.abs_diff_eq(&other.theta, epsilon)
        }
    }

    impl RelativeEq for State {
        fn default_max_relative() -> Self::Epsilon {
            Self::default_epsilon()
        }

        fn relative_eq(
            &self,
            other: &Self,
            epsilon: Self::Epsilon,
            max_relative: Self::Epsilon,
        ) -> bool {
            self.x.relative_eq(&other.x, epsilon, max_relative)
                && self.y.relative_eq(&other.y, epsilon, max_relative)
                && self.theta.relative_eq(&other.theta, epsilon, max_relative)
        }
    }
}
