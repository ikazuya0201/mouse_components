use core::iter::Chain;
use core::marker::PhantomData;

use super::straight_generator::{
    AngleOverallCalculator, AngleStraightCalculatorGenerator, StraightTrajectory,
    StraightTrajectoryGenerator,
};
use super::trajectory::{LengthTarget, Target};
use crate::traits::Math;
use uom::si::{
    f32::{Angle, AngularAcceleration, AngularJerk, AngularVelocity, Length, Time, Velocity},
    ratio::ratio,
};

pub enum SlalomDirection {
    Right,
    Left,
}

pub enum SlalomKind {
    Search90(SlalomDirection),
    FastRun45(SlalomDirection),
    FastRun90(SlalomDirection),
    FastRun135(SlalomDirection),
    FastRun180(SlalomDirection),
    FastRunDiagonal90(SlalomDirection),
}

pub struct SlalomParameters {
    pub xy: [(Length, Length); 3],
    pub theta: Angle,
    pub v_ref: Velocity,
}

pub struct SlalomGenerator<M> {
    dtheta: AngularVelocity,
    ddtheta: AngularAcceleration,
    dddtheta: AngularJerk,
    period: Time,
    parameters_map: fn(SlalomKind) -> SlalomParameters,
    _phantom: PhantomData<fn() -> M>,
}

impl<M> SlalomGenerator<M> {
    pub fn new(
        dtheta: AngularVelocity,
        ddtheta: AngularAcceleration,
        dddtheta: AngularJerk,
        period: Time,
        parameters_map: fn(SlalomKind) -> SlalomParameters,
    ) -> Self {
        Self {
            dtheta,
            ddtheta,
            dddtheta,
            period,
            parameters_map,
            _phantom: PhantomData,
        }
    }
}

pub type SlalomTrajectory<M> =
    Chain<Chain<StraightTrajectory, CurveTrajectory<M>>, StraightTrajectory>;

impl<M> SlalomGenerator<M>
where
    M: Math,
{
    pub fn generate_slalom(&self, kind: SlalomKind, v: Velocity) -> SlalomTrajectory<M> {
        let params = (self.parameters_map)(kind);
        let straight1 = StraightTrajectoryGenerator::<M>::generate_constant(
            Default::default(),
            Default::default(),
            params.xy[0].0,
            params.xy[0].1,
            v,
            self.period,
        );
        let curve = self.generate_curve(
            params.xy[0].0,
            params.xy[0].1,
            Default::default(),
            params.theta,
            v,
            params.v_ref,
        );
        let straight2 = StraightTrajectoryGenerator::<M>::generate_constant(
            params.xy[1].0,
            params.xy[1].1,
            params.xy[2].0,
            params.xy[2].1,
            v,
            self.period,
        );
        straight1.chain(curve).chain(straight2)
    }

    fn generate_curve(
        &self,
        x: Length,
        y: Length,
        theta: Angle,
        theta_distance: Angle,
        v: Velocity,
        v_ref: Velocity,
    ) -> CurveTrajectory<M> {
        let k = (v / v_ref).get::<ratio>();
        let angle_generator = AngleStraightCalculatorGenerator::<M>::new(
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
        CurveTrajectory::new(angle_fn, t_end, self.period, x, y, v)
    }
}

#[derive(Clone)]
pub struct CurveTrajectory<M> {
    angle_calculator: AngleOverallCalculator,
    t: Time,
    t_end: Time,
    period: Time,
    x: Length,
    y: Length,
    v: Velocity,
    _phantom: PhantomData<fn() -> M>,
}

impl<M> CurveTrajectory<M> {
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
            _phantom: PhantomData,
        }
    }
}

impl<M: Math> Iterator for CurveTrajectory<M> {
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
            let (sin, cos) = M::sincos(targets[i].x);
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
        let jx = -self.v
            * (cos_theta * AngularAcceleration::from(target.v * target.v) + sin_theta * target.a);

        let vy = self.v * sin_theta;
        let ay = self.v * cos_theta * target.v;
        let jy = self.v
            * (-sin_theta * AngularAcceleration::from(target.v * target.v) + cos_theta * target.a);

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
pub use tests::parameters_map;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::math::MathFake;
    use proptest::prelude::*;
    use uom::si::{
        angle::{degree, radian},
        angular_acceleration::{degree_per_second_squared, radian_per_second_squared},
        angular_jerk::{degree_per_second_cubed, radian_per_second_cubed},
        angular_velocity::{degree_per_second, radian_per_second},
        length::{meter, millimeter},
        time::second,
        velocity::meter_per_second,
    };

    const EPSILON: f32 = 1e-4;

    fn get_curve_trajectory(
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
    ) -> CurveTrajectory<MathFake> {
        let dtheta = AngularVelocity::new::<radian_per_second>(tv);
        let ddtheta = AngularAcceleration::new::<radian_per_second_squared>(ta);
        let dddtheta = AngularJerk::new::<radian_per_second_cubed>(tj);
        let v_ref = Velocity::new::<meter_per_second>(v_ref);
        let period = Time::new::<second>(period);

        let generator = SlalomGenerator::new(dtheta, ddtheta, dddtheta, period, parameters_map);
        let v_target = Velocity::new::<meter_per_second>(v_target);
        generator.generate_curve(
            Length::new::<meter>(x),
            Length::new::<meter>(y),
            Angle::new::<degree>(theta),
            Angle::new::<degree>(theta_dist),
            v_target,
            v_ref,
        )
    }

    pub fn parameters_map(kind: SlalomKind) -> SlalomParameters {
        match kind {
            SlalomKind::Search90(dir) => match dir {
                SlalomDirection::Left => SlalomParameters {
                    xy: [
                        (Length::new::<millimeter>(5.00001), Default::default()),
                        (
                            Length::new::<millimeter>(45.0),
                            Length::new::<millimeter>(40.0),
                        ),
                        (
                            Length::new::<millimeter>(45.0),
                            Length::new::<millimeter>(45.0),
                        ),
                    ],
                    v_ref: Velocity::new::<meter_per_second>(0.24159),
                    theta: Angle::new::<degree>(90.0),
                },
                SlalomDirection::Right => SlalomParameters {
                    xy: [
                        (Length::new::<millimeter>(5.00001), Default::default()),
                        (
                            Length::new::<millimeter>(45.0),
                            Length::new::<millimeter>(-40.0),
                        ),
                        (
                            Length::new::<millimeter>(45.0),
                            Length::new::<millimeter>(-45.0),
                        ),
                    ],
                    v_ref: Velocity::new::<meter_per_second>(0.24159),
                    theta: Angle::new::<degree>(-90.0),
                },
            },
            _ => unreachable!(),
        }
    }

    #[test]
    fn test_slalom_generator() {
        let generator = SlalomGenerator::<MathFake>::new(
            AngularVelocity::new::<degree_per_second>(540.0),
            AngularAcceleration::new::<degree_per_second_squared>(6480.0),
            AngularJerk::new::<degree_per_second_cubed>(216000.0),
            Time::new::<second>(0.001),
            parameters_map,
        );
        let v_target = Velocity::new::<meter_per_second>(0.5);

        let trajectory =
            generator.generate_slalom(SlalomKind::Search90(SlalomDirection::Left), v_target);
        for target in trajectory {
            println!(
                "{},{},{},{}",
                target.x.x.get::<meter>(),
                target.y.x.get::<meter>(),
                target.x.v.get::<meter_per_second>(),
                target.y.v.get::<meter_per_second>()
            );
            let v = target.x.v * target.theta.x.get::<radian>().cos()
                + target.y.v * target.theta.x.get::<radian>().sin();
            assert!(
                (v.abs().get::<meter_per_second>() - v_target.get::<meter_per_second>()).abs()
                    < EPSILON
            );
        }
    }

    proptest! {
        #[ignore]
        #[test]
        fn test_curve_generator(
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
            let trajectory = get_curve_trajectory(tv,ta,tj,v_ref,period,v_target,x,y,theta,theta_dist);
            for target in trajectory {
                let v = target.x.v * target.theta.x.get::<radian>().cos() + target.y.v * target.theta.x.get::<radian>().sin();
                prop_assert!((v.abs().get::<meter_per_second>() - v_target).abs() < EPSILON);
            }
        }
    }
}
