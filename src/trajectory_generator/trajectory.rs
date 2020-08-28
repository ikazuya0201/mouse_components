use crate::quantities::f32::{
    Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, Jerk, Length, Velocity,
};

#[derive(Clone, Copy, Debug, Default)]
pub struct Target {
    pub x: LengthTarget,
    pub y: LengthTarget,
    pub theta: AngleTarget,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LengthTarget {
    pub x: Length,
    pub v: Velocity,
    pub a: Acceleration,
    pub j: Jerk,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AngleTarget {
    pub x: Angle,
    pub v: AngularVelocity,
    pub a: AngularAcceleration,
    pub j: AngularJerk,
}
