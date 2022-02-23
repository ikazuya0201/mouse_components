use core::marker::PhantomData;

#[allow(unused_imports)]
use micromath::F32Ext;
use serde::{Deserialize, Serialize};
use typed_builder::TypedBuilder;
use uom::{
    si::{
        angle::radian,
        angular_velocity::radian_per_second,
        electric_potential::volt,
        f32::{
            Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity,
            AvailableEnergy, ElectricPotential, Frequency, Jerk, Length, Ratio, Time, Velocity,
        },
        jerk::meter_per_second_cubed,
        length::millimeter,
        ratio::ratio,
        time::second,
        Quantity, ISQ, SI,
    },
    typenum::*,
    Kind,
};

use crate::estimate::State;

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct MotorOutput {
    pub left: ElectricPotential,
    pub right: ElectricPotential,
}

type GainType = Quantity<ISQ<Z0, Z0, N2, Z0, Z0, Z0, Z0, dyn Kind>, SI<f32>, f32>;
type BType = Quantity<ISQ<N2, Z0, Z0, Z0, Z0, Z0, Z0, dyn Kind>, SI<f32>, f32>;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Target {
    pub x: LengthTarget,
    pub y: LengthTarget,
    pub theta: AngleTarget,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LengthTarget {
    pub x: Length,
    pub v: Velocity,
    pub a: Acceleration,
    pub j: Jerk,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AngleTarget {
    pub x: Angle,
    pub v: AngularVelocity,
    pub a: AngularAcceleration,
    pub j: AngularJerk,
}

#[derive(Clone, PartialEq, Debug)]
pub struct ControlTarget {
    pub v: Velocity,
    pub a: Acceleration,
    pub omega: AngularVelocity,
    pub alpha: AngularAcceleration,
}

#[derive(Debug, TypedBuilder)]
pub struct Tracker {
    #[builder(default, setter(skip))]
    xi: Velocity,
    period: Time,
    xi_threshold: Velocity,
    zeta: f32,
    #[builder(setter(transform = |value: f32| BType{ value, dimension: PhantomData, units: PhantomData }))]
    b: BType,
}

impl Tracker {
    pub fn track(
        &mut self,
        state: &State,
        target: &Target,
        input: &TrackingInput,
    ) -> (ControlTarget, ControlTarget) {
        let sin_th = state.theta.x.value.sin();
        let cos_th = state.theta.x.value.cos();

        let vv = state.x.v * cos_th + state.y.v * sin_th;
        let va = state.x.a * cos_th + state.y.a * sin_th;

        let TrackingInput { ux, uy, dux, duy } = *input;

        let dxi = ux * cos_th + uy * sin_th;
        self.xi += self.period * dxi;

        let sin_th_r = target.theta.x.value.sin();
        let cos_th_r = target.theta.x.value.cos();
        let vr = target.x.v * cos_th_r + target.y.v * sin_th_r;

        let (uv, uw, duv, duw) =
            if vr.abs() > self.xi_threshold && self.xi.abs() > self.xi_threshold {
                let uv = self.xi;
                let uw = AngularVelocity::from((uy * cos_th - ux * sin_th) / self.xi);
                let duv = dxi;
                let duw = -AngularAcceleration::from(
                    (2.0 * dxi * uw + dux * sin_th - duy * cos_th) / self.xi,
                );
                (uv, uw, duv, duw)
            } else {
                let theta_d = normalize_angle(target.theta.x - state.theta.x);
                let cos_th_d = theta_d.value.cos();
                let xd = target.x.x - state.x.x;
                let yd = target.y.x - state.y.x;

                let wr = target.theta.v;

                let k1 = 2.0
                    * self.zeta
                    * AngularVelocity::new::<radian_per_second>(
                        (wr * wr + self.b * vr * vr).value.sqrt(),
                    );
                let k2 = self.b;
                let k3 = k1;

                let uv = vr * cos_th_d + k1 * (xd * cos_th + yd * sin_th);
                let uw =
                    wr + AngularVelocity::from(
                        k2 * vr * (-xd * sin_th + yd * cos_th) * sinc(theta_d.value),
                    ) + AngularVelocity::from(k3 * theta_d);
                (
                    uv,
                    uw,
                    target.x.a * cos_th_r + target.y.a * sin_th_r,
                    target.theta.a,
                )
            };

        (
            ControlTarget {
                v: uv,
                a: duv,
                omega: uw,
                alpha: duw,
            },
            ControlTarget {
                v: vv,
                a: va,
                omega: state.theta.v,
                alpha: state.theta.a,
            },
        )
    }
}

// normalize angle to [-pi, pi].
fn normalize_angle(angle: Angle) -> Angle {
    use core::f32::consts::{PI, TAU};

    let raw_angle = angle.value.rem_euclid(TAU);

    Angle::new::<radian>(if raw_angle > PI {
        raw_angle - TAU
    } else {
        raw_angle
    })
}

// calculate sin(x)/x
fn sinc(x: f32) -> f32 {
    let xx = x * x;
    let xxxx = xx * xx;
    xxxx * xxxx / 362880.0 - xxxx * xx / 5040.0 + xxxx / 120.0 - xx / 6.0 + 1.0
}

#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct ControlParameters {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub model_k: f32,
    pub model_t1: f32,
}

#[derive(Debug, TypedBuilder)]
pub struct Controller {
    trans_params: ControlParameters,
    rot_params: ControlParameters,
    #[builder(default, setter(skip))]
    trans_sum: f32,
    #[builder(default, setter(skip))]
    rot_sum: f32,
    period: Time,
}

impl Controller {
    fn control_each(
        r: f32,
        dr: f32,
        y: f32,
        dy: f32,
        params: &ControlParameters,
        period: Time,
        error_sum: &mut f32,
    ) -> ElectricPotential {
        let vol_f = (dr * params.model_t1 + r) / params.model_k;
        let vol_p = params.kp * (r - y);
        let vol_i = params.ki * *error_sum;
        let vol_d = params.kd * (dr - dy);
        *error_sum += (r - y) * period.get::<second>();
        ElectricPotential::new::<volt>(vol_f + vol_p + vol_i + vol_d)
    }

    pub fn control(&mut self, r: &ControlTarget, y: &ControlTarget) -> MotorOutput {
        let vol_t = Self::control_each(
            r.v.value,
            r.a.value,
            y.v.value,
            y.a.value,
            &self.trans_params,
            self.period,
            &mut self.trans_sum,
        );
        let vol_r = Self::control_each(
            r.omega.value,
            r.alpha.value,
            y.omega.value,
            y.alpha.value,
            &self.rot_params,
            self.period,
            &mut self.rot_sum,
        );
        let left_motor_voltage = vol_t - vol_r;
        let right_motor_voltage = vol_t + vol_r;

        MotorOutput {
            left: left_motor_voltage,
            right: right_motor_voltage,
        }
    }
}

#[derive(Debug, TypedBuilder)]
pub struct NavigationController {
    #[builder(setter(transform = |value: f32| GainType{ value, dimension: PhantomData, units: PhantomData }))]
    gain: GainType,
    #[builder(setter(transform = |value: f32| Frequency{ value, dimension: PhantomData, units: PhantomData }))]
    dgain: Frequency,
}

impl NavigationController {
    pub fn navigate(&self, state: &State, target: &Target) -> TrackingInput {
        TrackingInput {
            ux: target.x.a
                + self.dgain * (target.x.v - state.x.v)
                + self.gain * (target.x.x - state.x.x),
            uy: target.y.a
                + self.dgain * (target.y.v - state.y.v)
                + self.gain * (target.y.x - state.y.x),
            dux: target.x.j
                + self.dgain * (target.x.a - state.x.a)
                + self.gain * (target.x.v - state.x.v),
            duy: target.y.j
                + self.dgain * (target.y.a - state.y.a)
                + self.gain * (target.y.v - state.y.v),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackingInput {
    pub ux: Acceleration,
    pub uy: Acceleration,
    pub dux: Jerk,
    pub duy: Jerk,
}

fn remquof(value: Length, base: Length) -> (i8, Length) {
    let iwidth = base.get::<millimeter>() as usize;
    let ivalue = value.get::<millimeter>() as usize;
    let div = ivalue / iwidth;
    (div as i8, value - base * div as f32)
}

#[derive(Debug, TypedBuilder)]
pub struct SupervisoryController {
    #[builder(setter(transform = |value: f32| Frequency{ value, dimension: PhantomData, units: PhantomData }))]
    margin: Frequency,
    avoidance_distance: Length,
    square_width: Length,
}

impl SupervisoryController {
    pub fn supervise(&self, input: &TrackingInput, state: &State) -> TrackingInput {
        let (x, y) = self.nearest_pillar(state);
        let a1 = 2.0 * (state.x.x - x);
        let a2 = 2.0 * (state.y.x - y);
        let b = 2.0 * (state.x.v * state.x.v + state.y.v * state.y.v)
            + 4.0 * self.margin * ((state.x.x - x) * state.x.v + (state.y.x - y) * state.y.v)
            + self.margin
                * self.margin
                * ((state.x.x - x) * (state.x.x - x) + (state.y.x - y) * (state.y.x - y)
                    - self.avoidance_distance * self.avoidance_distance);
        self.apply_constraints(input, a1, a2, b)
    }

    fn nearest_pillar(&self, state: &State) -> (Length, Length) {
        let (divx, remx) = remquof(state.x.x, self.square_width);
        let (divy, remy) = remquof(state.y.x, self.square_width);

        let (x, y) = match (
            2.0 * remx > self.square_width,
            2.0 * remy > self.square_width,
        ) {
            (true, true) => (divx + 1, divy + 1),
            (true, false) => (divx + 1, divy),
            (false, true) => (divx, divy + 1),
            (false, false) => (divx, divy),
        };
        (x as f32 * self.square_width, y as f32 * self.square_width)
    }

    fn apply_constraints(
        &self,
        &TrackingInput { ux, uy, dux, duy }: &TrackingInput,
        a1: Length,
        a2: Length,
        b: AvailableEnergy,
    ) -> TrackingInput {
        assert!(!a1.is_nan());
        assert!(!a2.is_nan());
        assert!(!b.is_nan());
        if a1 * ux + a2 * uy + b >= AvailableEnergy::default() {
            return TrackingInput { ux, uy, dux, duy };
        }
        let (ux2, uy2) = if a2 != Length::default() {
            let a = -a1 / a2;
            let b = -b / a2;
            let c1 = a * a + Ratio::new::<ratio>(1.0);
            let c2 = 2.0 * (a * b - ux - a * uy);
            let ux2 = -c2 / (2.0 * c1);
            let uy2 = a * ux2 + b;
            (ux2, uy2)
        } else if a1 != Length::default() {
            let a = -a2 / a1;
            let b = -b / a1;
            let c1 = a * a + Ratio::new::<ratio>(1.0);
            let c2 = 2.0 * (a * b - uy - a * ux);
            let uy2 = -c2 / (2.0 * c1);
            let ux2 = a * uy2 + b;
            (ux2, uy2)
        } else {
            unreachable!("Infeasible")
        };
        assert!(!ux2.is_nan());
        assert!(!uy2.is_nan());

        let to_polar = |x: f32, y: f32| ((x * x + y * y).sqrt(), y.atan2(x));
        let (r, t) = to_polar(ux.value, uy.value);
        let (r2, t2) = to_polar(ux2.value, uy2.value);
        let (dr, dt) = to_polar(dux.value, duy.value);
        let dr2 = Jerk::new::<meter_per_second_cubed>(dr * r2 / r);
        let dt2 = dt + t2 - t;
        let dux2 = dr2 * dt2.cos();
        let duy2 = dr2 * dt2.sin();
        TrackingInput {
            ux: ux2,
            uy: uy2,
            dux: dux2,
            duy: duy2,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_normalize_angle() {
        use uom::si::angle::degree;

        let test_cases = vec![
            (45.0, 45.0),
            (180.0, 180.0),
            (-45.0, -45.0),
            (-300.0, 60.0),
            (-660.0, 60.0),
        ];

        for (angle, expected) in test_cases {
            let angle = Angle::new::<degree>(angle);
            let expected = Angle::new::<degree>(expected);
            assert_relative_eq!(
                normalize_angle(angle).value,
                expected.value,
                epsilon = 0.001
            );
        }
    }

    #[test]
    fn test_remquof() {
        let test_cases = vec![(200.0, 2, 20.0), (714.0, 7, 84.0)];
        let base = Length::new::<millimeter>(90.0);
        for (value, exp_div, exp_rem) in test_cases {
            let (div, rem) = remquof(Length::new::<millimeter>(value), base);
            assert_eq!(div, exp_div);
            assert_relative_eq!(rem.get::<millimeter>(), exp_rem, epsilon = 0.001);
        }
    }
}
