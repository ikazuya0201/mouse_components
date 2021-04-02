use core::f32::consts::PI;
use core::iter::Chain;
use core::marker::PhantomData;

#[cfg(test)]
use proptest_derive::Arbitrary;
use uom::si::{
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, Jerk, Length, Time,
        Velocity,
    },
    ratio::ratio,
};

use super::straight_generator::{
    AngleOverallCalculator, AngleStraightCalculatorGenerator, StraightTrajectory,
    StraightTrajectoryGenerator,
};
use super::trajectory::{LengthTarget, MoveTarget, ShiftTrajectory, Target};
use crate::traits::Math;
use crate::types::data::Pose;

/// An enum for specifying the direction of slaloms.
#[cfg_attr(test, derive(Arbitrary))]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SlalomDirection {
    Right,
    Left,
}

/// An enum for specifying types of slaloms.
#[cfg_attr(test, derive(Arbitrary))]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SlalomKind {
    Search90,
    FastRun45,
    FastRun45Rev,
    FastRun90,
    FastRun135,
    FastRun135Rev,
    FastRun180,
    FastRunDiagonal90,
}

/// Parameters for slalom.
#[derive(Clone, PartialEq, Debug)]
pub struct SlalomParameters {
    pub l_start: Length,
    pub l_end: Length,
    pub x_curve_end: Length,
    pub y_curve_end: Length,
    pub theta: Angle,
    pub v_ref: Velocity,
    pub dtheta: AngularVelocity,
    pub ddtheta: AngularAcceleration,
    pub dddtheta: AngularJerk,
}

/// A trait that generates parameters for slalom.
pub trait SlalomParametersGenerator {
    fn generate(&self, kind: SlalomKind, direction: SlalomDirection) -> SlalomParameters;
}

pub struct SlalomGenerator<M, Generator> {
    period: Time,
    parameters_generator: Generator,
    #[allow(unused)]
    straight_generator: StraightTrajectoryGenerator<M>,
}

impl<M, Generator> SlalomGenerator<M, Generator> {
    pub fn new(
        period: Time,
        parameters_generator: Generator,
        v_max: Velocity,
        a_max: Acceleration,
        j_max: Jerk,
    ) -> Self
    where
        Generator: SlalomParametersGenerator,
    {
        Self {
            period,
            parameters_generator,
            straight_generator: StraightTrajectoryGenerator::new(v_max, a_max, j_max, period),
        }
    }
}

pub type SlalomTrajectory<M> =
    Chain<Chain<StraightTrajectory, CurveTrajectory<M>>, ShiftTrajectory<StraightTrajectory, M>>;

impl<M, Generator> SlalomGenerator<M, Generator>
where
    M: Math,
    Generator: SlalomParametersGenerator,
{
    pub fn generate_constant_slalom(
        &self,
        kind: SlalomKind,
        dir: SlalomDirection,
        v: Velocity,
    ) -> SlalomTrajectory<M> {
        let params = self.parameters_generator.generate(kind, dir);
        let straight1 =
            StraightTrajectoryGenerator::<M>::generate_constant(params.l_start, v, self.period);
        let curve = self.generate_curve(
            params.l_start,
            Default::default(),
            Default::default(),
            params.theta,
            v,
            params.v_ref,
            params.dtheta,
            params.ddtheta,
            params.dddtheta,
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

    #[allow(unused)]
    //TODO: Write test.
    pub fn generate_slalom_with_terminal_velocity(
        &self,
        kind: SlalomKind,
        dir: SlalomDirection,
        v_start: Velocity,
        v_end: Velocity,
    ) -> (SlalomTrajectory<M>, Velocity) {
        let params = self.parameters_generator.generate(kind, dir);
        let (start_straight, middle_velocity) = self
            .straight_generator
            .generate_with_terminal_velocity(params.l_start, v_start, v_end);
        let curve = self.generate_curve(
            params.l_start,
            Default::default(),
            Default::default(),
            params.theta,
            middle_velocity,
            params.v_ref,
            params.dtheta,
            params.ddtheta,
            params.dddtheta,
        );
        let (end_straight, terminal_velocity) = self
            .straight_generator
            .generate_with_terminal_velocity(params.l_end, middle_velocity, v_end);
        let end_straight = ShiftTrajectory::new(
            Pose {
                x: params.x_curve_end,
                y: params.y_curve_end,
                theta: params.theta,
            },
            end_straight,
        );
        (
            start_straight.chain(curve).chain(end_straight),
            terminal_velocity,
        )
    }

    #[inline]
    fn generate_curve(
        &self,
        x: Length,
        y: Length,
        theta: Angle,
        theta_distance: Angle,
        v: Velocity,
        v_ref: Velocity,
        dtheta: AngularVelocity,
        ddtheta: AngularAcceleration,
        dddtheta: AngularJerk,
    ) -> CurveTrajectory<M> {
        let k = (v / v_ref).get::<ratio>();
        let angle_generator = AngleStraightCalculatorGenerator::<M>::new(
            k * dtheta,
            k * k * ddtheta,
            k * k * k * dddtheta,
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

impl<M> Clone for CurveTrajectory<M> {
    fn clone(&self) -> Self {
        Self {
            angle_calculator: self.angle_calculator.clone(),
            t: self.t.clone(),
            t_end: self.t_end.clone(),
            period: self.period.clone(),
            x: self.x.clone(),
            y: self.y.clone(),
            v: self.v.clone(),
            _phantom: PhantomData,
        }
    }
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

        let cs = [0.21132487, 0.7886751];
        let bs = [0.5, 0.5];
        let mut delta_x = 0.0;
        let mut delta_y = 0.0;
        for i in 0..2 {
            let (sin, cos) = M::sincos(self.angle_calculator.calculate(t + cs[i] * self.period).x);
            delta_x += bs[i] * cos;
            delta_y += bs[i] * sin;
        }

        let x = self.x;
        let y = self.y;

        //gauss legendre (s=2)
        self.x += self.v * self.period * delta_x;
        self.y += self.v * self.period * delta_y;

        let target = self.angle_calculator.calculate(t);
        let (sin_theta, cos_theta) = M::sincos(target.x);

        let vx = self.v * cos_theta;
        let ax = -self.v * sin_theta * target.v;
        let jx = -self.v
            * (cos_theta * AngularAcceleration::from(target.v * target.v) + sin_theta * target.a);

        let vy = self.v * sin_theta;
        let ay = self.v * cos_theta * target.v;
        let jy = self.v
            * (-sin_theta * AngularAcceleration::from(target.v * target.v) + cos_theta * target.a);

        Some(Target::Moving(MoveTarget {
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
        }))
    }

    fn advance_by(&mut self, n: usize) -> Result<(), usize> {
        let t = self.t;
        self.t += self.period * n as f32;

        let update_pos = |this: &mut Self, dt: Time| {
            //gauss legendre (s=3)
            let cs = [0.112701654, 0.5, 0.88729835];
            let bs = [0.2777778, 0.44444445, 0.2777778];
            let mut delta_x = 0.0;
            let mut delta_y = 0.0;
            for i in 0..3 {
                let (sin, cos) = M::sincos(this.angle_calculator.calculate(t + cs[i] * dt).x);
                delta_x += bs[i] * cos;
                delta_y += bs[i] * sin;
            }

            this.x += this.v * dt * delta_x;
            this.y += this.v * dt * delta_y;
        };
        if self.t >= self.t_end {
            update_pos(self, self.t - self.t_end);
            Err(n - ((self.t - self.t_end) / self.period).get::<uom::si::ratio::ratio>() as usize)
        } else {
            update_pos(self, self.period * n as f32);
            Ok(())
        }
    }
}

pub const DEFAULT_DTHETA: AngularVelocity = AngularVelocity {
    value: 3.0 * PI,
    dimension: PhantomData,
    units: PhantomData,
};

pub const DEFAULT_DDTHETA: AngularAcceleration = AngularAcceleration {
    value: 36.0 * PI,
    dimension: PhantomData,
    units: PhantomData,
};

pub const DEFAULT_DDDTHETA: AngularJerk = AngularJerk {
    value: 1200.0 * PI,
    dimension: PhantomData,
    units: PhantomData,
};

fn into_parameters(
    l_start: f32,
    l_end: f32,
    x_curve_end: f32,
    y_curve_end: f32,
    v_ref: f32,
    theta: f32,
    dir: SlalomDirection,
) -> SlalomParameters {
    use uom::si::{angle::degree, length::millimeter, velocity::millimeter_per_second};
    use SlalomDirection::*;

    let (y_curve_end, theta) = match dir {
        Left => (y_curve_end, theta),
        Right => (-y_curve_end, -theta),
    };
    SlalomParameters {
        l_start: Length::new::<millimeter>(l_start),
        l_end: Length::new::<millimeter>(l_end),
        x_curve_end: Length::new::<millimeter>(x_curve_end),
        y_curve_end: Length::new::<millimeter>(y_curve_end),
        v_ref: Velocity::new::<millimeter_per_second>(v_ref),
        theta: Angle::new::<degree>(theta),
        dtheta: DEFAULT_DTHETA,
        ddtheta: DEFAULT_DDTHETA,
        dddtheta: DEFAULT_DDDTHETA,
    }
}

pub struct DefaultSlalomParametersGenerator;

impl SlalomParametersGenerator for DefaultSlalomParametersGenerator {
    fn generate(&self, kind: SlalomKind, direction: SlalomDirection) -> SlalomParameters {
        use SlalomKind::*;

        match kind {
            Search90 => into_parameters(5.000001, 5.0, 45.0, 40.0, 241.59, 90.0, direction),
            FastRun45 => into_parameters(2.57365, 21.2132, 75.0, 30.0, 411.636, 45.0, direction),
            FastRun45Rev => into_parameters(
                21.2132, 2.57365, 93.63957, 29.99996, 411.636, 45.0, direction,
            ),
            FastRun90 => into_parameters(20.0, 20.0, 90.0, 70.0, 422.783, 90.0, direction),
            FastRun135 => into_parameters(21.8627, 14.1421, 55.0, 80.0, 353.609, 135.0, direction),
            FastRun135Rev => into_parameters(
                14.1421, 21.8627, 47.27907, 80.00015, 353.609, 135.0, direction,
            ),
            FastRun180 => into_parameters(24.0, 24.0, 24.0, 90.0, 412.228, 180.0, direction),
            FastRunDiagonal90 => {
                into_parameters(15.6396, 15.6396, 63.6396, 48.0, 289.908, 90.0, direction)
            }
        }
    }
}

pub struct SlalomParametersGeneratorWithFrontOffset {
    search90_param_right: SlalomParameters,
    search90_param_left: SlalomParameters,
}

impl SlalomParametersGeneratorWithFrontOffset {
    pub fn new(front_offset: Length) -> Self {
        let x = front_offset.get::<uom::si::length::millimeter>();
        let gen_params = |l_start: f32,
                          l_end: f32,
                          x_curve_end: f32,
                          y_curve_end: f32,
                          v_ref: f32,
                          theta: f32| {
            (
                into_parameters(
                    l_start,
                    l_end,
                    x_curve_end,
                    y_curve_end,
                    v_ref,
                    theta,
                    SlalomDirection::Right,
                ),
                into_parameters(
                    l_start,
                    l_end,
                    x_curve_end,
                    y_curve_end,
                    v_ref,
                    theta,
                    SlalomDirection::Left,
                ),
            )
        };
        let search90_params = if x <= 5.0 {
            gen_params(5.000001 - x, 5.0 + x, 45.0 - x, 40.0, 241.59, 90.0)
        } else {
            gen_params(
                0.0,
                x + x,
                45.0 - x,
                45.0 - x,
                (45.0 - x) * 241.59 / 40.0,
                90.0,
            )
        };
        Self {
            search90_param_right: search90_params.0,
            search90_param_left: search90_params.1,
        }
    }
}

impl SlalomParametersGenerator for SlalomParametersGeneratorWithFrontOffset {
    fn generate(&self, kind: SlalomKind, direction: SlalomDirection) -> SlalomParameters {
        match kind {
            SlalomKind::Search90 => match direction {
                SlalomDirection::Left => self.search90_param_left.clone(),
                SlalomDirection::Right => self.search90_param_right.clone(),
            },
            _ => DefaultSlalomParametersGenerator.generate(kind, direction),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::math::MathFake;
    use proptest::prelude::*;
    use uom::si::{
        acceleration::meter_per_second_squared,
        angle::{degree, radian},
        angular_acceleration::radian_per_second_squared,
        angular_jerk::radian_per_second_cubed,
        angular_velocity::radian_per_second,
        jerk::meter_per_second_cubed,
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

        let generator = SlalomGenerator::new(
            period,
            DefaultSlalomParametersGenerator,
            Velocity::new::<meter_per_second>(1.0),
            Acceleration::new::<meter_per_second_squared>(1.0),
            Jerk::new::<meter_per_second_cubed>(1.0),
        );
        let v_target = Velocity::new::<meter_per_second>(v_target);
        generator.generate_curve(
            Length::new::<meter>(x),
            Length::new::<meter>(y),
            Angle::new::<degree>(theta),
            Angle::new::<degree>(theta_dist),
            v_target,
            v_ref,
            dtheta,
            ddtheta,
            dddtheta,
        )
    }

    proptest! {
        #[test]
        fn test_slalom_generator(kind: SlalomKind, dir: SlalomDirection) {
            use approx::assert_relative_eq;

            let generator = SlalomGenerator::<MathFake, DefaultSlalomParametersGenerator>::new(
                Time::new::<second>(0.001),
                DefaultSlalomParametersGenerator,
                Velocity::new::<meter_per_second>(1.0),
                Acceleration::new::<meter_per_second_squared>(1.0),
                Jerk::new::<meter_per_second_cubed>(1.0),
            );
            let v_target = Velocity::new::<meter_per_second>(0.5);

            let trajectory = generator.generate_constant_slalom(
                kind,
                dir,
                v_target,
            );
            for target in trajectory {
                let target = target.moving().unwrap();
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
                let target = target.moving().unwrap();
                let v = target.x.v * target.theta.x.get::<radian>().cos() + target.y.v * target.theta.x.get::<radian>().sin();
                prop_assert!((v.abs().get::<meter_per_second>() - v_target).abs() < EPSILON);
            }
        }
    }
}
