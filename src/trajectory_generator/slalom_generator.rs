use libm::{cosf, sinf};
use quantities::{Angle, AngularAcceleration, AngularJerk, AngularSpeed, Distance, Speed, Time};

use super::trajectory::Target;
use crate::utils::vector::Vector2;

use super::straight_generator::StraightFunctionGenerator;

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
    ) -> impl Iterator<Item = Vector2<Target<Distance>>> {
        let k = v / self.v_ref;
        let angle_generator = StraightFunctionGenerator::<Angle>::new(
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

struct SlalomTrajectory<F>
where
    F: Fn(Time) -> Target<Angle>,
{
    angle_fn: F,
    t: Time,
    t_end: Time,
    period: Time,
    x: Distance,
    y: Distance,
    v: Speed,
}

impl<F> SlalomTrajectory<F>
where
    F: Fn(Time) -> Target<Angle>,
{
    pub fn new(
        angle_fn: F,
        t_end: Time,
        period: Time,
        x_start: Distance,
        y_start: Distance,
        v: Speed,
    ) -> Self {
        Self {
            angle_fn,
            t: Default::default(),
            t_end,
            period,
            x: x_start,
            y: y_start,
            v,
        }
    }
}

impl<F> Iterator for SlalomTrajectory<F>
where
    F: Fn(Time) -> Target<Angle>,
{
    type Item = Vector2<Target<Distance>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.t > self.t_end {
            return None;
        }
        let t = self.t;
        self.t += self.period;
        let targets = [
            (self.angle_fn)(t),
            (self.angle_fn)(t + self.period / 2.0),
            (self.angle_fn)(t + self.period),
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

        Some(Vector2 {
            x: Target {
                x,
                v: vx,
                a: ax,
                j: jx,
            },
            y: Target {
                x: y,
                v: vy,
                a: ay,
                j: jy,
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_slalom_generator_search90() {
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
            Angle::from_degree(180.0),
            Angle::from_degree(90.0),
            v_target,
        );

        for target in trajectory {
            let vx = target.x.v.as_meter_per_second();
            let vy = target.y.v.as_meter_per_second();
            let v = Speed::from_meter_per_second((vx * vx + vy * vy).sqrt());
            assert_relative_eq!(v, v_target);
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
            assert_relative_eq!(v, v_target);
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
            assert_relative_eq!(v, v_target);
        }
    }
}
