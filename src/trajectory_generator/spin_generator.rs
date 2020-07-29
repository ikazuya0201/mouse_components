use quantities::{Angle, AngularAcceleration, AngularJerk, AngularSpeed, Time};

use super::straight_generator::StraightFunctionGenerator;
use super::trajectory::{SubTarget, Target};

pub struct SpinGenerator {
    function_generator: StraightFunctionGenerator<Angle>,
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
            function_generator: StraightFunctionGenerator::new(
                max_angular_speed,
                max_angular_acceleration,
                max_angular_jerk,
            ),
            period,
        }
    }

    pub fn generate(
        &self,
        theta_start: Angle,
        theta_distance: Angle,
    ) -> impl Iterator<Item = Target> {
        let (trajectory_fn, t_end) = self.function_generator.generate(
            theta_start,
            theta_distance,
            Default::default(),
            Default::default(),
        );
        SpinTrajectory::new(trajectory_fn, t_end, self.period)
    }
}

pub struct SpinTrajectory<F> {
    trajectory_fn: F,
    t: Time,
    t_end: Time,
    period: Time,
}

impl<F> SpinTrajectory<F> {
    fn new(trajectory_fn: F, t_end: Time, period: Time) -> Self {
        Self {
            trajectory_fn,
            t: Default::default(),
            t_end,
            period,
        }
    }
}

impl<F> Iterator for SpinTrajectory<F>
where
    F: Fn(Time) -> SubTarget<Angle>,
{
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
            theta: (self.trajectory_fn)(t),
        })
    }
}
