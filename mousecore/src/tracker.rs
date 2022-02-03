//! An implementation of [Tracker](crate::robot::Tracker).

use core::marker::PhantomData;

#[allow(unused_imports)]
use micromath::F32Ext;
use serde::{Deserialize, Serialize};
use uom::si::{
    angle::radian,
    f32::{Acceleration, Angle, AngularAcceleration, AngularVelocity, Frequency, Time, Velocity},
    frequency::hertz,
    Quantity, ISQ, SI,
};
use uom::{typenum::*, Kind};

use super::robot::Tracker as ITracker;
use super::trajectory_generators::Target;
use crate::types::data::RobotState;
use crate::utils::builder::RequiredFieldEmptyError;
use crate::{Construct, Deconstruct};

type GainType = Quantity<ISQ<Z0, Z0, N2, Z0, Z0, Z0, Z0, dyn Kind>, SI<f32>, f32>;
type BType = Quantity<ISQ<N2, Z0, Z0, Z0, Z0, Z0, Z0, dyn Kind>, SI<f32>, f32>;

#[derive(Clone, PartialEq, Debug)]
pub struct ControlTarget {
    pub v: Velocity,
    pub a: Acceleration,
    pub omega: AngularVelocity,
    pub alpha: AngularAcceleration,
}

pub trait Controller {
    type Error;

    fn control(&mut self, r: &ControlTarget, y: &ControlTarget) -> Result<(), Self::Error>;
    /// Stops all actuators in controller.
    fn stop(&mut self);
}

/// An implementation of [Tracker](crate::robot::Tracker).
pub struct Tracker<Controller> {
    gain: GainType,
    dgain: Frequency,
    xi: Velocity,
    period: Time,
    xi_threshold: Velocity,
    controller: Controller,
    zeta: f32,
    b: BType,
}

impl<Controller> Tracker<Controller> {
    pub fn release(self) -> Controller {
        let Self { controller, .. } = self;
        controller
    }
}

/// Config for [Tracker].
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct TrackerConfig {
    pub gain: f32,
    pub dgain: f32,
    pub period: Time,
    pub valid_control_lower_bound: Velocity,
    pub low_zeta: f32,
    pub low_b: f32,
}

impl<ControllerType, Config, State, Resource> Construct<Config, State, Resource>
    for Tracker<ControllerType>
where
    ControllerType: Construct<Config, State, Resource> + Controller,
    Config: AsRef<TrackerConfig>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let controller = ControllerType::construct(config, state, resource);
        let config = config.as_ref();
        TrackerBuilder::new()
            .controller(controller)
            .gain(config.gain)
            .dgain(config.dgain)
            .period(config.period)
            .valid_control_lower_bound(config.valid_control_lower_bound)
            .low_zeta(config.low_zeta)
            .low_b(config.low_b)
            .build()
            .expect("Should never panic")
    }
}

impl<Controller, State, Resource> Deconstruct<State, Resource> for Tracker<Controller>
where
    State: Default,
    Controller: Deconstruct<State, Resource>,
{
    fn deconstruct(self) -> (State, Resource) {
        let controller = self.release();
        controller.deconstruct()
    }
}

impl<Controller> core::fmt::Debug for Tracker<Controller> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "Tracker{{ xi:{:?} }}", self.xi)
    }
}

impl<ControllerType> ITracker<RobotState, Target> for Tracker<ControllerType>
where
    ControllerType: Controller,
{
    type Error = ControllerType::Error;

    fn track(&mut self, state: &RobotState, target: &Target) -> Result<(), Self::Error> {
        let sin_th = state.theta.x.value.sin();
        let cos_th = state.theta.x.value.cos();

        let vv = state.x.v * cos_th + state.y.v * sin_th;
        let va = state.x.a * cos_th + state.y.a * sin_th;

        //calculate control input for (x,y)
        let ux = target.x.a
            + self.dgain * (target.x.v - state.x.v)
            + self.gain * (target.x.x - state.x.x.mean);
        let uy = target.y.a
            + self.dgain * (target.y.v - state.y.v)
            + self.gain * (target.y.x - state.y.x.mean);
        let dux = target.x.j
            + self.dgain * (target.x.a - state.x.a)
            + self.gain * (target.x.v - state.x.v);
        let duy = target.y.j
            + self.dgain * (target.y.a - state.y.a)
            + self.gain * (target.y.v - state.y.v);

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
                let xd = target.x.x - state.x.x.mean;
                let yd = target.y.x - state.y.x.mean;

                let wr = target.theta.v;

                let k1 = 2.0
                    * self.zeta
                    * AngularVelocity::from(crate::utils::math::sqrt(wr * wr + self.b * vr * vr));
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

        self.controller.control(
            &ControlTarget {
                v: uv,
                a: duv,
                omega: uw,
                alpha: duw,
            },
            &ControlTarget {
                v: vv,
                a: va,
                omega: state.theta.v,
                alpha: state.theta.a,
            },
        )
    }

    fn stop(&mut self) {
        self.controller.stop();
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

pub struct TrackerBuilder<Controller> {
    gain: Option<GainType>,
    dgain: Option<Frequency>,
    xi_threshold: Option<Velocity>,
    controller: Option<Controller>,
    period: Option<Time>,
    zeta: Option<f32>,
    b: Option<BType>,
}

impl<ControllerType> Default for TrackerBuilder<ControllerType> {
    fn default() -> Self {
        Self::new()
    }
}

impl<ControllerType> TrackerBuilder<ControllerType> {
    pub fn new() -> Self {
        Self {
            gain: None,
            dgain: None,
            xi_threshold: None,
            controller: None,
            period: None,
            zeta: None,
            b: None,
        }
    }

    pub fn gain(&mut self, gain: f32) -> &mut Self {
        self.gain = Some(GainType {
            value: gain,
            dimension: PhantomData,
            units: PhantomData,
        });
        self
    }

    pub fn dgain(&mut self, dgain: f32) -> &mut Self {
        self.dgain = Some(Frequency::new::<hertz>(dgain));
        self
    }

    pub fn valid_control_lower_bound(&mut self, xi_threshold: Velocity) -> &mut Self {
        self.xi_threshold = Some(xi_threshold);
        self
    }

    pub fn controller(&mut self, controller: ControllerType) -> &mut Self
    where
        ControllerType: Controller,
    {
        self.controller = Some(controller);
        self
    }

    pub fn period(&mut self, period: Time) -> &mut Self {
        self.period = Some(period);
        self
    }

    pub fn low_zeta(&mut self, zeta: f32) -> &mut Self {
        self.zeta = Some(zeta);
        self
    }

    pub fn low_b(&mut self, b: f32) -> &mut Self {
        self.b = Some(BType {
            value: b,
            ..Default::default()
        });
        self
    }

    pub fn build(&mut self) -> Result<Tracker<ControllerType>, RequiredFieldEmptyError> {
        Ok(Tracker {
            gain: get_or_err!(self.gain),
            dgain: get_or_err!(self.dgain),
            xi_threshold: get_or_err!(self.xi_threshold),
            controller: get_or_err!(self.controller),
            period: get_or_err!(self.period),
            zeta: get_or_err!(self.zeta),
            b: get_or_err!(self.b),
            xi: Default::default(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use uom::si::{time::second, velocity::meter_per_second};

    struct IController;

    impl Controller for IController {
        type Error = core::convert::Infallible;

        fn control(&mut self, _r: &ControlTarget, _y: &ControlTarget) -> Result<(), Self::Error> {
            Ok(())
        }

        fn stop(&mut self) {
            unimplemented!()
        }
    }

    fn build_tracker() -> Tracker<IController> {
        TrackerBuilder::new()
            .gain(1.0)
            .dgain(1.0)
            .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.001))
            .period(Time::new::<second>(0.001))
            .controller(IController)
            .low_zeta(1.0)
            .low_b(1e-3)
            .build()
            .unwrap()
    }

    #[test]
    fn test_build() {
        let _tracker = build_tracker();
    }

    #[test]
    fn test_normalize_angle() {
        use approx::assert_relative_eq;
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
}
