use libm::{cosf, sinf};
use quantities::{Angle, AngularAcceleration, AngularJerk, AngularSpeed, Distance, Speed, Time};

use super::trajectory::{SubTarget, Target};

use super::straight_generator::{OverallCalculator, StraightCalculatorGenerator};

pub struct SlalomGenerator {
    dtheta: AngularSpeed,
    ddtheta: AngularAcceleration,
    dddtheta: AngularJerk,
    v_ref: Speed,
    period: Time,
}

impl SlalomGenerator {
    pub fn new(
        dtheta: AngularSpeed,
        ddtheta: AngularAcceleration,
        dddtheta: AngularJerk,
        v_ref: Speed,
        period: Time,
    ) -> Self {
        Self {
            dtheta,
            ddtheta,
            dddtheta,
            v_ref,
            period,
        }
    }

    pub fn generate(
        &self,
        x: Distance,
        y: Distance,
        theta: Angle,
        theta_distance: Angle,
        v: Speed,
    ) -> SlalomTrajectory {
        let k = v / self.v_ref;
        let angle_generator = StraightCalculatorGenerator::<Angle>::new(
            k * self.dtheta,
            k * k * self.ddtheta,
            k * k * k * self.dddtheta,
        );
        let (angle_fn, t_end) = angle_generator.generate(
            theta,
            theta_distance,
            AngularSpeed::from_radian_per_second(0.0),
            AngularSpeed::from_radian_per_second(0.0),
        );
        SlalomTrajectory::new(angle_fn, t_end, self.period, x, y, v)
    }
}

#[derive(Clone)]
pub struct SlalomTrajectory {
    angle_calculator: OverallCalculator<Angle>,
    t: Time,
    t_end: Time,
    period: Time,
    x: Distance,
    y: Distance,
    v: Speed,
}

impl SlalomTrajectory {
    pub fn new(
        angle_calculator: OverallCalculator<Angle>,
        t_end: Time,
        period: Time,
        x_start: Distance,
        y_start: Distance,
        v: Speed,
    ) -> Self {
        Self {
            angle_calculator,
            t: Default::default(),
            t_end,
            period,
            x: x_start,
            y: y_start,
            v,
        }
    }
}

impl Iterator for SlalomTrajectory {
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        if self.t > self.t_end {
            return None;
        }
        let t = self.t;
        self.t += self.period;
        let targets = [
            self.angle_calculator.calculate(t),
            self.angle_calculator.calculate(t + self.period / 2.0),
            self.angle_calculator.calculate(t + self.period),
        ];

        let mut sin_theta = [0.0; 3];
        let mut cos_theta = [0.0; 3];
        for i in 0..3 {
            sin_theta[i] = sinf(targets[i].x.as_radian());
            cos_theta[i] = cosf(targets[i].x.as_radian());
        }

        let x = self.x;
        let y = self.y;
        self.x += self.v * self.period * (cos_theta[0] + 4.0 * cos_theta[1] + cos_theta[2]) / 6.0;
        self.y += self.v * self.period * (sin_theta[0] + 4.0 * sin_theta[1] + sin_theta[2]) / 6.0;

        let target = targets[0].clone();
        let cos_theta = cos_theta[0];
        let sin_theta = sin_theta[0];

        let vx = self.v * cos_theta;
        let ax = -self.v * sin_theta * target.v;
        let jx = -self.v * (cos_theta * target.v * target.v + sin_theta * target.a);

        let vy = self.v * sin_theta;
        let ay = self.v * cos_theta * target.v;
        let jy = self.v * (-sin_theta * target.v * target.v + cos_theta * target.a);

        Some(Target {
            x: SubTarget {
                x,
                v: vx,
                a: ax,
                j: jx,
            },
            y: SubTarget {
                x: y,
                v: vy,
                a: ay,
                j: jy,
            },
            theta: target,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::EPSILON;
    use quantities::Quantity;

    #[test]
    fn test_slalom_generator_search90() {
        use std::f32::consts::PI;

        let dtheta = AngularSpeed::from_radian_per_second(3.0 * PI);
        let ddtheta = AngularAcceleration::from_radian_per_second_squared(36.0 * PI);
        let dddtheta = AngularJerk::from_radian_per_second_cubed(1200.0 * PI);
        let v_ref = Speed::from_meter_per_second(0.24159);
        let period = Time::from_seconds(0.001);

        let generator = SlalomGenerator::new(dtheta, ddtheta, dddtheta, v_ref, period);
        let v_target = Speed::from_meter_per_second(0.2);
        let trajectory = generator.generate(
            Distance::from_meters(0.0),
            Distance::from_meters(0.0),
            Angle::from_degree(90.0),
            Angle::from_degree(90.0),
            v_target,
        );

        for target in trajectory {
            let v = target.x.v * target.theta.x.cos() + target.y.v * target.theta.x.sin();
            assert!((v.abs() - v_target).abs().as_meter_per_second() < EPSILON);
        }
    }

    #[test]
    fn test_slalom_generator_run180() {
        use std::f32::consts::PI;

        let dtheta = AngularSpeed::from_radian_per_second(3.0 * PI);
        let ddtheta = AngularAcceleration::from_radian_per_second_squared(36.0 * PI);
        let dddtheta = AngularJerk::from_radian_per_second_cubed(1200.0 * PI);
        let v_ref = Speed::from_meter_per_second(0.24);
        let period = Time::from_seconds(0.001);

        let generator = SlalomGenerator::new(dtheta, ddtheta, dddtheta, v_ref, period);
        let v_target = Speed::from_meter_per_second(0.6);
        let trajectory = generator.generate(
            Distance::from_meters(0.0),
            Distance::from_meters(0.0),
            Angle::from_degree(90.0),
            Angle::from_degree(180.0),
            v_target,
        );

        for target in trajectory {
            let vx = target.x.v.as_meter_per_second();
            let vy = target.y.v.as_meter_per_second();
            let v = Speed::from_meter_per_second((vx * vx + vy * vy).sqrt());
            assert!((v - v_target).abs().as_meter_per_second() < EPSILON);
        }
    }

    #[test]
    fn test_slalom_generator_run45() {
        use std::f32::consts::PI;

        let dtheta = AngularSpeed::from_radian_per_second(3.0 * PI);
        let ddtheta = AngularAcceleration::from_radian_per_second_squared(36.0 * PI);
        let dddtheta = AngularJerk::from_radian_per_second_cubed(1200.0 * PI);
        let v_ref = Speed::from_meter_per_second(0.24);
        let period = Time::from_seconds(0.001);

        let generator = SlalomGenerator::new(dtheta, ddtheta, dddtheta, v_ref, period);
        let v_target = Speed::from_meter_per_second(0.6);
        let trajectory = generator.generate(
            Distance::from_meters(0.0),
            Distance::from_meters(0.0),
            Angle::from_degree(0.0),
            Angle::from_degree(45.0),
            v_target,
        );

        for target in trajectory {
            let vx = target.x.v.as_meter_per_second();
            let vy = target.y.v.as_meter_per_second();
            let v = Speed::from_meter_per_second((vx * vx + vy * vy).sqrt());
            assert!((v - v_target).abs().as_meter_per_second() < EPSILON);
        }
    }
}
