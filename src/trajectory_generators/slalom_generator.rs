use core::f32::consts::PI;
use core::iter::Chain;
use core::marker::PhantomData;

#[allow(unused_imports)]
use micromath::F32Ext;
#[cfg(test)]
use proptest_derive::Arbitrary;
use serde::{Deserialize, Serialize};
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
use super::trajectory::{LengthTarget, ShiftTrajectory, Target};
use crate::types::data::Pose;

/// An enum for specifying the direction of slaloms.
#[cfg_attr(test, derive(Arbitrary))]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum SlalomDirection {
    Right,
    Left,
}

/// An enum for specifying types of slaloms.
#[cfg_attr(test, derive(Arbitrary))]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
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
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
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

pub struct SlalomGenerator<Generator> {
    period: Time,
    parameters_generator: Generator,
    straight_generator: StraightTrajectoryGenerator,
}

impl<Generator> SlalomGenerator<Generator> {
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

pub type SlalomTrajectory =
    Chain<Chain<StraightTrajectory, CurveTrajectory>, ShiftTrajectory<StraightTrajectory>>;

impl<Generator> SlalomGenerator<Generator>
where
    Generator: SlalomParametersGenerator,
{
    pub fn generate_constant_slalom(
        &self,
        kind: SlalomKind,
        dir: SlalomDirection,
        v: Velocity,
    ) -> SlalomTrajectory {
        let params = self.parameters_generator.generate(kind, dir);
        let straight1 =
            StraightTrajectoryGenerator::generate_constant(params.l_start, v, self.period);
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
            StraightTrajectoryGenerator::generate_constant(params.l_end, v, self.period),
        );
        straight1.chain(curve).chain(straight2)
    }

    //TODO: Write test.
    pub fn generate_slalom_with_terminal_velocity(
        &self,
        kind: SlalomKind,
        dir: SlalomDirection,
        v_start: Velocity,
        v_end: Velocity,
    ) -> (SlalomTrajectory, Velocity) {
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
    #[allow(clippy::too_many_arguments)]
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
    ) -> CurveTrajectory {
        let k = (v / v_ref).get::<ratio>();
        let angle_generator = AngleStraightCalculatorGenerator::new(
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

pub struct CurveTrajectory {
    angle_calculator: AngleOverallCalculator,
    t: Time,
    t_end: Time,
    period: Time,
    x: Length,
    y: Length,
    v: Velocity,
}

impl Clone for CurveTrajectory {
    fn clone(&self) -> Self {
        Self {
            angle_calculator: self.angle_calculator.clone(),
            t: self.t,
            t_end: self.t_end,
            period: self.period,
            x: self.x,
            y: self.y,
            v: self.v,
        }
    }
}

impl CurveTrajectory {
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

impl Iterator for CurveTrajectory {
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
            let angle = self.angle_calculator.calculate(t + cs[i] * self.period).x;
            let sin = angle.value.sin();
            let cos = angle.value.cos();
            delta_x += bs[i] * cos;
            delta_y += bs[i] * sin;
        }

        let x = self.x;
        let y = self.y;

        //gauss legendre (s=2)
        self.x += self.v * self.period * delta_x;
        self.y += self.v * self.period * delta_y;

        let target = self.angle_calculator.calculate(t);
        let sin_theta = target.x.value.sin();
        let cos_theta = target.x.value.cos();

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

    #[cfg(nightly)]
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

/// Default implementation of [SlalomParametersGenerator].
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct DefaultSlalomParametersGenerator {
    search90: [SlalomParameters; 2],
    fast_run45: [SlalomParameters; 2],
    fast_run45_rev: [SlalomParameters; 2],
    fast_run90: [SlalomParameters; 2],
    fast_run135: [SlalomParameters; 2],
    fast_run135_rev: [SlalomParameters; 2],
    fast_run180: [SlalomParameters; 2],
    fast_run_diagonal90: [SlalomParameters; 2],
}

impl Default for DefaultSlalomParametersGenerator {
    fn default() -> Self {
        use uom::si::length::meter;

        Self::new(Length::new::<meter>(0.09), Default::default())
    }
}

impl DefaultSlalomParametersGenerator {
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

    pub fn new(square_width: Length, search_front_offset: Length) -> Self {
        use uom::si::length::meter;
        let search_front_offset = (search_front_offset / Length::new::<meter>(0.09)).value;
        Self {
            search90: if search_front_offset <= 0.055555556 {
                Self::create_params(
                    0.055555567 - search_front_offset,
                    0.055555556 + search_front_offset,
                    0.5 - search_front_offset,
                    0.44444445,
                    2.6843333,
                    90.0,
                    square_width,
                )
            } else {
                Self::create_params(
                    0.0,
                    search_front_offset + search_front_offset,
                    0.5 - search_front_offset,
                    0.5 - search_front_offset,
                    (0.5 - search_front_offset) * 6.03975,
                    90.0,
                    square_width,
                )
            },
            fast_run45: Self::create_params(
                0.156_441_4,
                0.368_032_1,
                0.739_762,
                0.239_762,
                3.333_333_3,
                45.0,
                square_width,
            ),
            fast_run45_rev: Self::create_params(
                0.363_547_5,
                0.156_441_2,
                0.950_039_5,
                0.242_932_8,
                3.333_333_3,
                45.0,
                square_width,
            ),
            fast_run90: Self::create_params(
                0.22222222,
                0.22222222,
                1.0,
                0.7777778,
                4.697589,
                90.0,
                square_width,
            ),
            fast_run135: Self::create_params(
                0.24291888,
                0.15713444,
                0.6111111,
                0.8888889,
                3.928989,
                135.0,
                square_width,
            ),
            fast_run135_rev: Self::create_params(
                0.15713444,
                0.24291888,
                0.52532303,
                0.88889056,
                3.928989,
                135.0,
                square_width,
            ),
            fast_run180: Self::create_params(
                0.26666668,
                0.26666668,
                0.26666668,
                1.0,
                4.5803113,
                180.0,
                square_width,
            ),
            fast_run_diagonal90: Self::create_params(
                0.17377333,
                0.17377333,
                0.70710665,
                0.53333336,
                3.2212,
                90.0,
                square_width,
            ),
        }
    }

    fn create_params(
        l_start: f32,
        l_end: f32,
        x_curve_end: f32,
        y_curve_end: f32,
        v_ref: f32,
        theta: f32,
        square_width: Length,
    ) -> [SlalomParameters; 2] {
        use uom::si::{angle::degree, f32::Frequency, frequency::hertz};

        let l_start = square_width * l_start;
        let l_end = square_width * l_end;
        let x_curve_end = square_width * x_curve_end;
        let y_curve_end = square_width * y_curve_end;
        let v_ref = square_width * Frequency::new::<hertz>(v_ref);
        let theta = Angle::new::<degree>(theta);
        let param = |y_curve_end, theta| SlalomParameters {
            l_start,
            l_end,
            x_curve_end,
            y_curve_end,
            v_ref,
            theta,
            dtheta: Self::DEFAULT_DTHETA,
            ddtheta: Self::DEFAULT_DDTHETA,
            dddtheta: Self::DEFAULT_DDDTHETA,
        };
        [param(y_curve_end, theta), param(-y_curve_end, -theta)]
    }
}

impl SlalomParametersGenerator for DefaultSlalomParametersGenerator {
    fn generate(&self, kind: SlalomKind, direction: SlalomDirection) -> SlalomParameters {
        use SlalomKind::*;

        let direction = match direction {
            SlalomDirection::Left => 0,
            SlalomDirection::Right => 1,
        };
        match kind {
            Search90 => self.search90[direction].clone(),
            FastRun45 => self.fast_run45[direction].clone(),
            FastRun45Rev => self.fast_run45_rev[direction].clone(),
            FastRun90 => self.fast_run90[direction].clone(),
            FastRun135 => self.fast_run135[direction].clone(),
            FastRun135Rev => self.fast_run135_rev[direction].clone(),
            FastRun180 => self.fast_run180[direction].clone(),
            FastRunDiagonal90 => self.fast_run_diagonal90[direction].clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
    ) -> CurveTrajectory {
        let dtheta = AngularVelocity::new::<radian_per_second>(tv);
        let ddtheta = AngularAcceleration::new::<radian_per_second_squared>(ta);
        let dddtheta = AngularJerk::new::<radian_per_second_cubed>(tj);
        let v_ref = Velocity::new::<meter_per_second>(v_ref);
        let period = Time::new::<second>(period);

        let generator = SlalomGenerator::new(
            period,
            DefaultSlalomParametersGenerator::default(),
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

            let generator = SlalomGenerator::< DefaultSlalomParametersGenerator>::new(
                Time::new::<second>(0.001),
                DefaultSlalomParametersGenerator::default(),
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
