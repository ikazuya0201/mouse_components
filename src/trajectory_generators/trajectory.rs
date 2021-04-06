use core::marker::PhantomData;

use serde::{Deserialize, Serialize};
use uom::si::f32::{
    Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, Jerk, Length, Time,
    Velocity,
};

use crate::traits::Math;
use crate::types::data::Pose;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Target {
    Moving(MoveTarget),
    Spin(AngleTarget),
}

impl Default for Target {
    fn default() -> Self {
        Target::Spin(AngleTarget::default())
    }
}

impl Target {
    pub fn moving(self) -> Option<MoveTarget> {
        match self {
            Target::Moving(target) => Some(target),
            _ => None,
        }
    }

    pub fn spin(self) -> Option<AngleTarget> {
        match self {
            Target::Spin(target) => Some(target),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MoveTarget {
    pub x: LengthTarget,
    pub y: LengthTarget,
    pub theta: AngleTarget,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LengthTarget {
    pub x: Length,
    pub v: Velocity,
    pub a: Acceleration,
    pub j: Jerk,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AngleTarget {
    pub x: Angle,
    pub v: AngularVelocity,
    pub a: AngularAcceleration,
    pub j: AngularJerk,
}

pub struct ShiftTrajectory<T, M> {
    pose: Pose,
    inner: T,
    _math: PhantomData<fn() -> M>,
}

impl<T, M> Clone for ShiftTrajectory<T, M>
where
    T: Clone,
{
    fn clone(&self) -> Self {
        Self {
            pose: self.pose.clone(),
            inner: self.inner.clone(),
            _math: PhantomData,
        }
    }
}

impl<T, M> ShiftTrajectory<T, M> {
    pub fn new(pose: Pose, inner: T) -> Self {
        Self {
            pose,
            inner,
            _math: PhantomData,
        }
    }
}

impl<T, M> ShiftTrajectory<T, M>
where
    M: Math,
{
    fn shift(&self, target: MoveTarget) -> MoveTarget {
        let (sin_th, cos_th) = M::sincos(self.pose.theta);
        MoveTarget {
            x: LengthTarget {
                x: target.x.x * cos_th - target.y.x * sin_th + self.pose.x,
                v: target.x.v * cos_th - target.y.v * sin_th,
                a: target.x.a * cos_th - target.y.a * sin_th,
                j: target.x.j * cos_th - target.y.j * sin_th,
            },
            y: LengthTarget {
                x: target.x.x * sin_th + target.y.x * cos_th + self.pose.y,
                v: target.x.v * sin_th + target.y.v * cos_th,
                a: target.x.a * sin_th + target.y.a * cos_th,
                j: target.x.j * sin_th + target.y.j * cos_th,
            },
            theta: AngleTarget {
                x: target.theta.x + self.pose.theta,
                v: target.theta.v,
                a: target.theta.a,
                j: target.theta.j,
            },
        }
    }
}

impl<T, M> Iterator for ShiftTrajectory<T, M>
where
    T: Iterator<Item = Target>,
    M: Math,
{
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        match self.inner.next()? {
            Target::Moving(target) => Some(Target::Moving(self.shift(target))),
            target => Some(target),
        }
    }

    fn advance_by(&mut self, n: usize) -> Result<(), usize> {
        self.inner.advance_by(n)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct StopTrajectory {
    t: Time,
    pose: Pose,
    period: Time,
    t_end: Time,
}

impl StopTrajectory {
    pub fn new(pose: Pose, period: Time, t_end: Time) -> Self {
        Self {
            t: Default::default(),
            pose,
            period,
            t_end,
        }
    }
}

impl Iterator for StopTrajectory {
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        if self.t < self.t_end {
            self.t += self.period;
            Some(Target::Moving(MoveTarget {
                x: LengthTarget {
                    x: self.pose.x,
                    ..Default::default()
                },
                y: LengthTarget {
                    x: self.pose.y,
                    ..Default::default()
                },
                theta: AngleTarget {
                    x: self.pose.theta,
                    ..Default::default()
                },
            }))
        } else {
            None
        }
    }

    fn advance_by(&mut self, n: usize) -> Result<(), usize> {
        self.t += self.period * n as f32;
        if self.t < self.t_end {
            Ok(())
        } else {
            Err(n - ((self.t - self.t_end) / self.period).get::<uom::si::ratio::ratio>() as usize)
        }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use uom::si::{
        acceleration::meter_per_second_squared, angle::degree,
        angular_acceleration::degree_per_second_squared, angular_jerk::degree_per_second_cubed,
        angular_velocity::degree_per_second, jerk::meter_per_second_cubed, length::meter,
        velocity::meter_per_second,
    };

    use super::*;

    #[test]
    fn test_shift_trajectory() {
        use crate::utils::math::MathFake;

        let target = Target::Moving(MoveTarget {
            x: LengthTarget {
                x: Length::new::<meter>(1.0),
                v: Velocity::new::<meter_per_second>(1.0),
                ..Default::default()
            },
            y: LengthTarget {
                x: Length::new::<meter>(1.0),
                v: Velocity::new::<meter_per_second>(1.0),
                ..Default::default()
            },
            theta: AngleTarget {
                x: Angle::new::<degree>(90.0),
                v: AngularVelocity::new::<degree_per_second>(360.0),
                ..Default::default()
            },
        });
        let pose = Pose {
            x: Length::new::<meter>(1.0),
            y: Length::new::<meter>(1.0),
            theta: Angle::new::<degree>(90.0),
        };
        let expected_target = Target::Moving(MoveTarget {
            x: LengthTarget {
                x: Length::new::<meter>(0.0),
                v: Velocity::new::<meter_per_second>(-1.0),
                ..Default::default()
            },
            y: LengthTarget {
                x: Length::new::<meter>(2.0),
                v: Velocity::new::<meter_per_second>(1.0),
                ..Default::default()
            },
            theta: AngleTarget {
                x: Angle::new::<degree>(180.0),
                v: AngularVelocity::new::<degree_per_second>(360.0),
                ..Default::default()
            },
        });
        let mut trajectory = ShiftTrajectory::<_, MathFake>::new(pose, vec![target].into_iter());
        assert_target_relative_eq(
            trajectory.next().unwrap().moving().unwrap(),
            expected_target.moving().unwrap(),
        );
    }

    fn assert_target_relative_eq(left: MoveTarget, right: MoveTarget) {
        assert_relative_eq!(left.x.x.get::<meter>(), right.x.x.get::<meter>());
        assert_relative_eq!(
            left.x.v.get::<meter_per_second>(),
            right.x.v.get::<meter_per_second>()
        );
        assert_relative_eq!(
            left.x.a.get::<meter_per_second_squared>(),
            right.x.a.get::<meter_per_second_squared>()
        );
        assert_relative_eq!(
            left.x.j.get::<meter_per_second_cubed>(),
            right.x.j.get::<meter_per_second_cubed>()
        );

        assert_relative_eq!(left.y.x.get::<meter>(), right.y.x.get::<meter>());
        assert_relative_eq!(
            left.y.v.get::<meter_per_second>(),
            right.y.v.get::<meter_per_second>()
        );
        assert_relative_eq!(
            left.y.a.get::<meter_per_second_squared>(),
            right.y.a.get::<meter_per_second_squared>()
        );
        assert_relative_eq!(
            left.y.j.get::<meter_per_second_cubed>(),
            right.y.j.get::<meter_per_second_cubed>()
        );

        assert_relative_eq!(left.theta.x.get::<degree>(), right.theta.x.get::<degree>());
        assert_relative_eq!(
            left.theta.v.get::<degree_per_second>(),
            right.theta.v.get::<degree_per_second>()
        );
        assert_relative_eq!(
            left.theta.a.get::<degree_per_second_squared>(),
            right.theta.a.get::<degree_per_second_squared>()
        );
        assert_relative_eq!(
            left.theta.j.get::<degree_per_second_cubed>(),
            right.theta.j.get::<degree_per_second_cubed>()
        );
    }
}
