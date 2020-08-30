use uom::si::f32::{Angle, AngularAcceleration, AngularJerk, AngularVelocity, Time};

use super::straight_generator::{AngleOverallCalculator, AngleStraightCalculatorGenerator};
use super::trajectory::Target;
use crate::traits::Math;

pub struct SpinGenerator<M> {
    function_generator: AngleStraightCalculatorGenerator<M>,
    period: Time,
}

impl<M> SpinGenerator<M> {
    pub fn new(
        max_angular_velocity: AngularVelocity,
        max_angular_acceleration: AngularAcceleration,
        max_angular_jerk: AngularJerk,
        period: Time,
    ) -> Self {
        Self {
            function_generator: AngleStraightCalculatorGenerator::new(
                max_angular_velocity,
                max_angular_acceleration,
                max_angular_jerk,
            ),
            period,
        }
    }
}

impl<M> SpinGenerator<M>
where
    M: Math,
{
    pub fn generate(&self, theta_start: Angle, theta_distance: Angle) -> SpinTrajectory {
        let (trajectory_fn, t_end) = self.function_generator.generate(
            theta_start,
            theta_distance,
            Default::default(),
            Default::default(),
        );
        SpinTrajectory::new(trajectory_fn, t_end, self.period)
    }
}

#[derive(Clone)]
pub struct SpinTrajectory {
    angle_calculator: AngleOverallCalculator,
    t: Time,
    t_end: Time,
    period: Time,
}

impl SpinTrajectory {
    fn new(angle_calculator: AngleOverallCalculator, t_end: Time, period: Time) -> Self {
        Self {
            angle_calculator,
            t: Default::default(),
            t_end,
            period,
        }
    }
}

impl Iterator for SpinTrajectory {
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        if self.t > self.t_end {
            return None;
        }
        let t = self.t;
        self.t += self.period;
        Some(Target {
            x: Default::default(),
            y: Default::default(),
            theta: self.angle_calculator.calculate(t),
        })
    }
}
