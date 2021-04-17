use uom::si::f32::{Angle, AngularAcceleration, AngularJerk, AngularVelocity, Time};

use super::straight_generator::{AngleOverallCalculator, AngleStraightCalculatorGenerator};
use super::trajectory::Target;

pub struct SpinGenerator {
    function_generator: AngleStraightCalculatorGenerator,
    period: Time,
}

impl SpinGenerator {
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

impl SpinGenerator {
    pub fn generate(&self, theta_distance: Angle) -> SpinTrajectory {
        let (trajectory_fn, t_end) = self.function_generator.generate(
            Default::default(),
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
            theta: self.angle_calculator.calculate(t),
            ..Default::default()
        })
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
