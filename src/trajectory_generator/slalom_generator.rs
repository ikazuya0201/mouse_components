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
    use proptest::prelude::*;
    use quantities::Quantity;

    const EPSILON: f32 = 1e-4;

    fn get_trajectory(
        tv: f32,
        ta: f32,
        tj: f32,
        v_ref: f32,
        period: f32,
        v_target: f32,
        x: f32,
        y: f32,
        theta: f32,
        theta_dist: f32,
    ) -> SlalomTrajectory {
        let dtheta = AngularSpeed::from_radian_per_second(tv);
        let ddtheta = AngularAcceleration::from_radian_per_second_squared(ta);
        let dddtheta = AngularJerk::from_radian_per_second_cubed(tj);
        let v_ref = Speed::from_meter_per_second(v_ref);
        let period = Time::from_seconds(period);

        let generator = SlalomGenerator::new(dtheta, ddtheta, dddtheta, v_ref, period);
        let v_target = Speed::from_meter_per_second(v_target);
        generator.generate(
            Distance::from_meters(x),
            Distance::from_meters(y),
            Angle::from_degree(theta),
            Angle::from_degree(theta_dist),
            v_target,
        )
    }

    proptest! {
        #[ignore]
        #[test]
        fn test_slalom_generator(
            tv in 0.1f32..1000.0,
            ta in 0.1f32..1000.0,
            tj in 0.1f32..1000.0,
            v_ref in 0.1f32..100.0,
            period in 0.001f32..0.01,
            v_target in 0.1f32..100.0,
            x in 0.0f32..288.0,
            y in 0.0f32..288.0,
            theta in 0.0f32..360.0,
            theta_dist in (-180.0f32..180.0).prop_filter("the absolute value is too small", |d| d.abs() >= 0.1),
        ) {
            let trajectory = get_trajectory(tv,ta,tj,v_ref,period,v_target,x,y,theta,theta_dist);
            for target in trajectory {
                let v = target.x.v * target.theta.x.cos() + target.y.v * target.theta.x.sin();
                prop_assert!((v.abs().as_meter_per_second() - v_target).abs() < EPSILON);
            }
        }
    }
}
