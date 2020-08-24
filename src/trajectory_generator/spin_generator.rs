use quantities::{Angle, AngularAcceleration, AngularJerk, AngularSpeed, Time};

use super::straight_generator::{OverallCalculator, StraightCalculatorGenerator};
use super::trajectory::Target;

pub struct SpinGenerator {
    function_generator: StraightCalculatorGenerator<Angle>,
    period: Time,
}

impl SpinGenerator {
    pub fn new(
        max_angular_speed: AngularSpeed,
        max_angular_acceleration: AngularAcceleration,
        max_angular_jerk: AngularJerk,
        period: Time,
    ) -> Self {
        Self {
            function_generator: StraightCalculatorGenerator::new(
                max_angular_speed,
                max_angular_acceleration,
                max_angular_jerk,
            ),
            period,
        }
    }

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
    angle_calculator: OverallCalculator<Angle>,
    t: Time,
    t_end: Time,
    period: Time,
}

impl SpinTrajectory {
    fn new(angle_calculator: OverallCalculator<Angle>, t_end: Time, period: Time) -> Self {
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
