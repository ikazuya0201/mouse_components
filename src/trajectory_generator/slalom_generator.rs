use core::iter::Chain;
use core::marker::PhantomData;

#[cfg(test)]
use proptest_derive::Arbitrary;
use uom::si::{
    f32::{Angle, AngularAcceleration, AngularJerk, AngularVelocity, Length, Time, Velocity},
    ratio::ratio,
};

use super::straight_generator::{
    AngleOverallCalculator, AngleStraightCalculatorGenerator, StraightTrajectory,
    StraightTrajectoryGenerator,
};
use super::trajectory::{LengthTarget, ShiftTrajectory, Target};
use crate::data_types::Pose;
use crate::traits::Math;

#[cfg_attr(test, derive(Arbitrary))]
#[derive(Debug)]
pub enum SlalomDirection {
    Right,
    Left,
}

#[cfg_attr(test, derive(Arbitrary))]
#[derive(Debug)]
pub enum SlalomKind {
    Search90,
    FastRun45,
    FastRun90,
    FastRun135,
    FastRun180,
    FastRunDiagonal90,
}

pub struct SlalomParameters {
    pub l_start: Length,
    pub l_end: Length,
    pub x_curve_end: Length,
    pub y_curve_end: Length,
    pub theta: Angle,
    pub v_ref: Velocity,
}

pub struct SlalomGenerator<M> {
    dtheta: AngularVelocity,
    ddtheta: AngularAcceleration,
    dddtheta: AngularJerk,
    period: Time,
    parameters_map: fn(SlalomKind, SlalomDirection) -> SlalomParameters,
    _phantom: PhantomData<fn() -> M>,
}

impl<M> SlalomGenerator<M> {
    pub fn new(
        dtheta: AngularVelocity,
        ddtheta: AngularAcceleration,
        dddtheta: AngularJerk,
        period: Time,
        parameters_map: fn(SlalomKind, SlalomDirection) -> SlalomParameters,
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
    Chain<Chain<StraightTrajectory, CurveTrajectory<M>>, ShiftTrajectory<StraightTrajectory, M>>;

impl<M> SlalomGenerator<M>
where
    M: Math,
{
    pub fn generate_slalom(
        &self,
        kind: SlalomKind,
        dir: SlalomDirection,
        v: Velocity,
    ) -> SlalomTrajectory<M> {
        let params = (self.parameters_map)(kind, dir);
        let straight1 =
            StraightTrajectoryGenerator::<M>::generate_constant(params.l_start, v, self.period);
        let curve = self.generate_curve(
            params.l_start,
            Default::default(),
            Default::default(),
            params.theta,
            v,
            params.v_ref,
        );
        let straight2 = ShiftTrajectory::new(
            Pose {
                x: params.x_curve_end,
                y: params.y_curve_end,
                theta: params.theta,
            },
            StraightTrajectoryGenerator::<M>::generate_constant(params.l_end, v, self.period),
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

pub fn slalom_parameters_map(kind: SlalomKind, dir: SlalomDirection) -> SlalomParameters {
    use uom::si::{angle::degree, length::millimeter, velocity::millimeter_per_second};
    use SlalomDirection::*;
    use SlalomKind::*;

    let params = match kind {
        Search90 => (5.000001, 5.0, 45.0, 40.0, 241.59, 90.0),
        FastRun45 => (2.57365, 21.2132, 75.0, 30.0, 411.636, 45.0),
        FastRun90 => (20.0, 20.0, 90.0, 70.0, 422.783, 90.0),
        FastRun135 => (21.8627, 14.1421, 55.0, 80.0, 353.609, 135.0),
        FastRun180 => (24.0, 24.0, 24.0, 90.0, 412.228, 180.0),
        FastRunDiagonal90 => (15.6396, 15.6396, 63.6396, 48.0, 289.908, 90.0),
    };
    let params = match dir {
        Left => params,
        Right => (params.0, params.1, params.2, -params.3, params.4, -params.5),
    };
    SlalomParameters {
        l_start: Length::new::<millimeter>(params.0),
        l_end: Length::new::<millimeter>(params.1),
        x_curve_end: Length::new::<millimeter>(params.2),
        y_curve_end: Length::new::<millimeter>(params.3),
        v_ref: Velocity::new::<millimeter_per_second>(params.4),
        theta: Angle::new::<degree>(params.5),
    }
}

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
        length::meter,
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

        let generator =
            SlalomGenerator::new(dtheta, ddtheta, dddtheta, period, slalom_parameters_map);
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

    proptest! {
        #[test]
        fn test_slalom_generator(kind: SlalomKind, dir: SlalomDirection) {
            use approx::assert_relative_eq;

            let generator = SlalomGenerator::<MathFake>::new(
                AngularVelocity::new::<degree_per_second>(540.0),
                AngularAcceleration::new::<degree_per_second_squared>(6480.0),
                AngularJerk::new::<degree_per_second_cubed>(216000.0),
                Time::new::<second>(0.001),
                slalom_parameters_map,
            );
            let v_target = Velocity::new::<meter_per_second>(0.5);

            let trajectory = generator.generate_slalom(
                kind,
                dir,
                v_target,
            );
            for target in trajectory {
                let v = target.x.v * target.theta.x.get::<radian>().cos()
                    + target.y.v * target.theta.x.get::<radian>().sin();
                assert_relative_eq!(
                    v.get::<meter_per_second>(),
                    v_target.get::<meter_per_second>(),
                    epsilon = EPSILON
                );
            }
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
