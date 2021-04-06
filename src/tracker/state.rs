use serde::{Deserialize, Serialize};
use uom::si::f32::{Acceleration, Angle, AngularAcceleration, AngularVelocity, Length, Velocity};

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RobotState {
    pub x: LengthState,
    pub y: LengthState,
    pub theta: AngleState,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LengthState {
    pub x: Length,
    pub v: Velocity,
    pub a: Acceleration,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AngleState {
    pub x: Angle,
    pub v: AngularVelocity,
    pub a: AngularAcceleration,
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::{AbsDiffEq, RelativeEq};
    use uom::si::{
        acceleration::meter_per_second_squared, angle::radian,
        angular_acceleration::radian_per_second_squared, angular_velocity::radian_per_second,
        length::meter, velocity::meter_per_second,
    };

    const V_RATIO: f32 = 100.0;
    const A_RATIO: f32 = 10000.0;

    impl AbsDiffEq for LengthState {
        type Epsilon = f32;

        fn default_epsilon() -> Self::Epsilon {
            std::f32::EPSILON
        }

        fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
            self.x
                .get::<meter>()
                .abs_diff_eq(&other.x.get::<meter>(), epsilon)
                && self
                    .v
                    .get::<meter_per_second>()
                    .abs_diff_eq(&other.v.get::<meter_per_second>(), epsilon * V_RATIO)
                && self.a.get::<meter_per_second_squared>().abs_diff_eq(
                    &other.a.get::<meter_per_second_squared>(),
                    epsilon * A_RATIO,
                )
        }
    }

    impl AbsDiffEq for AngleState {
        type Epsilon = f32;

        fn default_epsilon() -> Self::Epsilon {
            std::f32::EPSILON
        }

        fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
            self.x
                .get::<radian>()
                .abs_diff_eq(&other.x.get::<radian>(), epsilon)
                && self
                    .v
                    .get::<radian_per_second>()
                    .abs_diff_eq(&other.v.get::<radian_per_second>(), epsilon * V_RATIO)
                && self.a.get::<radian_per_second_squared>().abs_diff_eq(
                    &other.a.get::<radian_per_second_squared>(),
                    epsilon * A_RATIO,
                )
        }
    }

    impl RelativeEq for LengthState {
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
                .get::<meter>()
                .relative_eq(&other.x.get::<meter>(), epsilon, max_relative)
                && self.v.get::<meter_per_second>().relative_eq(
                    &other.v.get::<meter_per_second>(),
                    epsilon * V_RATIO,
                    max_relative,
                )
                && self.a.get::<meter_per_second_squared>().relative_eq(
                    &other.a.get::<meter_per_second_squared>(),
                    epsilon * A_RATIO,
                    max_relative,
                )
        }
    }

    impl RelativeEq for AngleState {
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
                .get::<radian>()
                .relative_eq(&other.x.get::<radian>(), epsilon, max_relative)
                && self.v.get::<radian_per_second>().relative_eq(
                    &other.v.get::<radian_per_second>(),
                    epsilon * V_RATIO,
                    max_relative,
                )
                && self.a.get::<radian_per_second_squared>().relative_eq(
                    &other.a.get::<radian_per_second_squared>(),
                    epsilon * A_RATIO,
                    max_relative,
                )
        }
    }

    impl AbsDiffEq for RobotState {
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

    impl RelativeEq for RobotState {
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
