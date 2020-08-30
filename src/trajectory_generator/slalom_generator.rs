use super::straight_generator::{AngleOverallCalculator, AngleStraightCalculatorGenerator};
use super::trajectory::{LengthTarget, Target};
use crate::quantities::f32::{
    Angle, AngularAcceleration, AngularJerk, AngularVelocity, Length, Time, Velocity,
};

pub struct SlalomGenerator {
    dtheta: AngularVelocity,
    ddtheta: AngularAcceleration,
    dddtheta: AngularJerk,
    v_ref: Velocity,
    period: Time,
}

impl SlalomGenerator {
    pub fn new(
        dtheta: AngularVelocity,
        ddtheta: AngularAcceleration,
        dddtheta: AngularJerk,
        v_ref: Velocity,
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
        x: Length,
        y: Length,
        theta: Angle,
        theta_distance: Angle,
        v: Velocity,
    ) -> SlalomTrajectory {
        let k = v / self.v_ref;
        let angle_generator = AngleStraightCalculatorGenerator::new(
            k * self.dtheta,
            k * k * self.ddtheta,
            k * k * k * self.dddtheta,
        );
        let (angle_fn, t_end) = angle_generator.generate(
            theta,
            theta_distance,
            Default::default(),
            Default::default(),
        );
        SlalomTrajectory::new(angle_fn, t_end, self.period, x, y, v)
    }
}

#[derive(Clone)]
pub struct SlalomTrajectory {
    angle_calculator: AngleOverallCalculator,
    t: Time,
    t_end: Time,
    period: Time,
    x: Length,
    y: Length,
    v: Velocity,
}

impl SlalomTrajectory {
    pub fn new(
        angle_calculator: AngleOverallCalculator,
        t_end: Time,
        period: Time,
        x_start: Length,
        y_start: Length,
        v: Velocity,
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
            let (sin, cos) = targets[i].x.sincos();
            sin_theta[i] = sin;
            cos_theta[i] = cos;
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
            x: LengthTarget {
                x,
                v: vx,
                a: ax,
                j: jx,
            },
            y: LengthTarget {
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
    use crate::quantities::{
        cubed_frequency::radian_per_second_cubed, dimensionless::degree,
        frequency::radian_per_second, length::meter, squared_frequency::radian_per_second_squared,
        time::second, velocity::meter_per_second,
    };
    use proptest::prelude::*;

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
        let dtheta = AngularVelocity::new::<radian_per_second>(tv);
        let ddtheta = AngularAcceleration::new::<radian_per_second_squared>(ta);
        let dddtheta = AngularJerk::new::<radian_per_second_cubed>(tj);
        let v_ref = Velocity::new::<meter_per_second>(v_ref);
        let period = Time::new::<second>(period);

        let generator = SlalomGenerator::new(dtheta, ddtheta, dddtheta, v_ref, period);
        let v_target = Velocity::new::<meter_per_second>(v_target);
        generator.generate(
            Length::new::<meter>(x),
            Length::new::<meter>(y),
            Angle::new::<degree>(theta),
            Angle::new::<degree>(theta_dist),
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
                prop_assert!((v.abs().get::<meter_per_second>() - v_target).abs() < EPSILON);
            }
        }
    }
}
