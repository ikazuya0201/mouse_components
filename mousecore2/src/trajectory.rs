pub mod slalom;
pub mod spin;
pub mod straight;

use core::marker::PhantomData;

#[allow(unused_imports)]
use micromath::F32Ext;
use typenum::{consts::*, operator_aliases::PartialQuot, type_operators::PartialDiv, Integer};
use uom::marker::Div;
use uom::si::f32::Time;
use uom::si::{Dimension, Quantity, ISQ, SI};

use crate::{
    control::{AngleTarget, LengthTarget, Target},
    wall::Pose,
};

#[allow(clippy::type_complexity)]
fn sqrt<D>(
    val: Quantity<D, SI<f32>, f32>,
) -> Quantity<
    ISQ<
        PartialQuot<D::L, P2>,
        PartialQuot<D::M, P2>,
        PartialQuot<D::T, P2>,
        PartialQuot<D::I, P2>,
        PartialQuot<D::Th, P2>,
        PartialQuot<D::N, P2>,
        PartialQuot<D::J, P2>,
    >,
    SI<f32>,
    f32,
>
where
    D: Dimension + ?Sized,
    D::L: PartialDiv<P2>,
    <D::L as PartialDiv<P2>>::Output: Integer,
    D::M: PartialDiv<P2>,
    <D::M as PartialDiv<P2>>::Output: Integer,
    D::T: PartialDiv<P2>,
    <D::T as PartialDiv<P2>>::Output: Integer,
    D::I: PartialDiv<P2>,
    <D::I as PartialDiv<P2>>::Output: Integer,
    D::Th: PartialDiv<P2>,
    <D::Th as PartialDiv<P2>>::Output: Integer,
    D::N: PartialDiv<P2>,
    <D::N as PartialDiv<P2>>::Output: Integer,
    D::J: PartialDiv<P2>,
    <D::J as PartialDiv<P2>>::Output: Integer,
    D::Kind: Div,
{
    Quantity {
        dimension: PhantomData,
        units: PhantomData,
        value: val.value.sqrt(),
    }
}

pub struct ShiftTrajectory<T> {
    pose: Pose,
    inner: T,
}

impl<T> Clone for ShiftTrajectory<T>
where
    T: Clone,
{
    fn clone(&self) -> Self {
        Self {
            pose: self.pose,
            inner: self.inner.clone(),
        }
    }
}

impl<T> ShiftTrajectory<T> {
    pub fn new(pose: Pose, inner: T) -> Self {
        Self { pose, inner }
    }
}

impl<T> ShiftTrajectory<T> {
    fn shift(&self, target: Target) -> Target {
        let sin_th = self.pose.theta.value.sin();
        let cos_th = self.pose.theta.value.cos();
        Target {
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

impl<T> Iterator for ShiftTrajectory<T>
where
    T: Iterator<Item = Target>,
{
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next().map(|item| self.shift(item))
    }

    #[cfg(nightly)]
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
            Some(Target {
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
            })
        } else {
            None
        }
    }

    #[cfg(nightly)]
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
        acceleration::meter_per_second_squared,
        angle::degree,
        angular_acceleration::degree_per_second_squared,
        angular_jerk::degree_per_second_cubed,
        angular_velocity::degree_per_second,
        f32::{Angle, AngularVelocity, Length, Velocity},
        jerk::meter_per_second_cubed,
        length::meter,
        velocity::meter_per_second,
    };

    use super::*;

    #[test]
    fn test_shift_trajectory() {
        let target = Target {
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
        };
        let pose = Pose {
            x: Length::new::<meter>(1.0),
            y: Length::new::<meter>(1.0),
            theta: Angle::new::<degree>(90.0),
        };
        let expected_target = Target {
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
        };
        let mut trajectory = ShiftTrajectory::new(pose, vec![target].into_iter());
        assert_target_relative_eq(trajectory.next().unwrap(), expected_target);
    }

    fn assert_target_relative_eq(left: Target, right: Target) {
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
