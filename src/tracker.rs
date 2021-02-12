mod state;

use core::marker::PhantomData;

use uom::si::{
    angle::radian,
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularVelocity, ElectricPotential, Frequency,
        Length, Time, Velocity,
    },
    frequency::hertz,
    Quantity, ISQ, SI,
};
use uom::{typenum::*, Kind};

use super::robot::Tracker as ITracker;
use super::trajectory_generators::{AngleTarget, MoveTarget, Target};
use crate::utils::builder::{ok_or, RequiredFieldEmptyError};
use crate::utils::math::{LibmMath, Math};
pub use state::{AngleState, LengthState, State};

pub trait Motor {
    fn apply(&mut self, electric_potential: ElectricPotential);
}

type GainType = Quantity<ISQ<Z0, Z0, N2, Z0, Z0, Z0, Z0, dyn Kind>, SI<f32>, f32>;
type BType = Quantity<ISQ<N2, Z0, Z0, Z0, Z0, Z0, Z0, dyn Kind>, SI<f32>, f32>;

macro_rules! controller_trait {
    ($name: ident: $t: ty, $dt: ty) => {
        pub trait $name {
            fn init(&mut self);
            fn calculate(&mut self, r: $t, dr: $dt, y: $t, dy: $dt) -> ElectricPotential;
        }
    };
}

controller_trait!(TranslationController: Velocity, Acceleration);
controller_trait!(RotationController: AngularVelocity, AngularAcceleration);

pub trait Logger {
    fn log(&self, state: &State, target: &Target);
}

pub struct NullLogger;

impl Logger for NullLogger {
    fn log(&self, _state: &State, _target: &Target) {}
}

pub struct Tracker<
    LM,
    RM,
    M,
    TC = crate::controllers::TranslationController,
    RC = crate::controllers::RotationController,
> {
    kx: GainType,
    kdx: Frequency,
    ky: GainType,
    kdy: Frequency,
    xi: Velocity,
    period: Time,
    xi_threshold: Velocity,
    fail_safe_distance: Length,
    translation_controller: TC,
    rotation_controller: RC,
    left_motor: LM,
    right_motor: RM,
    zeta: f32,
    b: BType,
    _phantom: PhantomData<fn() -> M>,
}

impl<LM, RM, M, TC, RC> core::fmt::Debug for Tracker<LM, RM, M, TC, RC> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "Tracker{{ xi:{:?} }}", self.xi)
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct FailSafeError {
    state: State,
    target: MoveTarget,
}

impl<LM, RM, M, TC, RC> ITracker<State, Target> for Tracker<LM, RM, M, TC, RC>
where
    LM: Motor,
    RM: Motor,
    TC: TranslationController,
    RC: RotationController,
    M: Math,
{
    type Error = FailSafeError;

    fn track(&mut self, state: &State, target: &Target) -> Result<(), Self::Error> {
        let (left, right) = match target {
            Target::Moving(target) => self.track_move(state, target)?,
            Target::Spin(target) => self.track_spin(state, target),
        };
        self.left_motor.apply(left);
        self.right_motor.apply(right);
        Ok(())
    }
}

impl<LM, RM, M, TC, RC> Tracker<LM, RM, M, TC, RC>
where
    LM: Motor,
    RM: Motor,
    TC: TranslationController,
    RC: RotationController,
    M: Math,
{
    pub fn stop(&mut self)
    where
        LM: Motor,
        RM: Motor,
    {
        self.left_motor.apply(Default::default());
        self.right_motor.apply(Default::default());
    }

    pub fn init(&mut self) {
        self.translation_controller.init();
        self.rotation_controller.init();
    }

    fn sinc(x: f32) -> f32 {
        let xx = x * x;
        let xxxx = xx * xx;
        xxxx * xxxx / 362880.0 - xxxx * xx / 5040.0 + xxxx / 120.0 - xx / 6.0 + 1.0
    }

    fn fail_safe(&mut self, state: &State, target: &MoveTarget) -> Result<(), FailSafeError> {
        let x_diff = state.x.x - target.x.x;
        let y_diff = state.y.x - target.y.x;

        let distance = M::sqrt(x_diff * x_diff + y_diff * y_diff);
        if distance >= self.fail_safe_distance {
            Err(FailSafeError {
                state: state.clone(),
                target: target.clone(),
            })
        } else {
            Ok(())
        }
    }

    fn track_move(
        &mut self,
        state: &State,
        target: &MoveTarget,
    ) -> Result<(ElectricPotential, ElectricPotential), FailSafeError> {
        self.fail_safe(state, target)?;

        let (sin_th, cos_th) = M::sincos(state.theta.x);

        let vv = state.x.v * cos_th + state.y.v * sin_th;
        let va = state.x.a * cos_th + state.y.a * sin_th;

        //calculate control input for (x,y)
        let ux =
            target.x.a + self.kdx * (target.x.v - state.x.v) + self.kx * (target.x.x - state.x.x);
        let uy =
            target.y.a + self.kdy * (target.y.v - state.y.v) + self.ky * (target.y.x - state.y.x);
        let dux =
            target.x.j + self.kdx * (target.x.a - state.x.a) + self.kx * (target.x.v - state.x.v);
        let duy =
            target.y.j + self.kdy * (target.y.a - state.y.a) + self.ky * (target.y.v - state.y.v);

        let dxi = ux * cos_th + uy * sin_th;
        let (uv, uw, duv, duw) = if self.xi.abs() > self.xi_threshold {
            let uv = self.xi;
            let uw = AngularVelocity::from((uy * cos_th - ux * sin_th) / self.xi);
            let duv = dxi;
            let duw = -AngularAcceleration::from(
                (2.0 * dxi * uw + dux * sin_th - duy * cos_th) / self.xi,
            );
            (uv, uw, duv, duw)
        } else {
            let (sin_th_r, cos_th_r) = M::sincos(target.theta.x);
            let theta_d = {
                let theta_d = target.theta.x - state.theta.x;
                Angle::new::<radian>(
                    M::rem_euclidf(theta_d.value, core::f32::consts::TAU) - core::f32::consts::PI,
                )
            };
            let cos_th_d = M::cos(theta_d);
            let xd = target.x.x - state.x.x;
            let yd = target.y.x - state.y.x;

            let vr = target.x.v * cos_th_r + target.y.v * sin_th_r;
            let wr = target.theta.v;

            let k1 = 2.0 * self.zeta * AngularVelocity::from(M::sqrt(wr * wr + self.b * vr * vr));
            let k2 = self.b;
            let k3 = k1;

            let e = xd * cos_th + yd * sin_th;
            let uv = vr * cos_th_d + k1 * e;
            let uw = wr
                + AngularVelocity::from(k2 * vr * e * Self::sinc(theta_d.get::<radian>()))
                + AngularVelocity::from(k3 * theta_d);
            (uv, uw, Default::default(), Default::default())
        };

        self.xi += self.period * dxi;
        //calculate motor voltage
        let vol_v = self.translation_controller.calculate(uv, duv, vv, va);
        let vol_w = self
            .rotation_controller
            .calculate(uw, duw, state.theta.v, state.theta.a);
        Ok((vol_v - vol_w, vol_v + vol_w))
    }

    fn track_spin(
        &mut self,
        state: &State,
        target: &AngleTarget,
    ) -> (ElectricPotential, ElectricPotential) {
        let (sin_th, cos_th) = M::sincos(state.theta.x);
        let vv = state.x.v * cos_th + state.y.v * sin_th;
        let va = state.x.a * cos_th + state.y.a * sin_th;

        let vol_v =
            self.translation_controller
                .calculate(Default::default(), Default::default(), vv, va);
        let vol_w =
            self.rotation_controller
                .calculate(target.v, target.a, state.theta.v, state.theta.a);

        (vol_v - vol_w, vol_v + vol_w)
    }
}

pub struct TrackerBuilder<TC, RC, LM, RM, M> {
    kx: Option<GainType>,
    kdx: Option<Frequency>,
    ky: Option<GainType>,
    kdy: Option<Frequency>,
    xi_threshold: Option<Velocity>,
    translation_controller: Option<TC>,
    rotation_controller: Option<RC>,
    left_motor: Option<LM>,
    right_motor: Option<RM>,
    period: Option<Time>,
    xi: Option<Velocity>,
    fail_safe_distance: Option<Length>,
    zeta: Option<f32>,
    b: Option<BType>,
    _math: PhantomData<fn() -> M>,
}

impl<TC, RC, LM, RM, M> TrackerBuilder<TC, RC, LM, RM, M> {
    pub fn new() -> Self {
        Self {
            kx: None,
            kdx: None,
            ky: None,
            kdy: None,
            xi_threshold: None,
            translation_controller: None,
            rotation_controller: None,
            left_motor: None,
            right_motor: None,
            period: None,
            xi: Some(Default::default()),
            fail_safe_distance: None,
            zeta: None,
            b: None,
            _math: PhantomData,
        }
    }

    pub fn kx(mut self, kx: f32) -> Self {
        self.kx = Some(GainType {
            value: kx,
            dimension: PhantomData,
            units: PhantomData,
        });
        self
    }

    pub fn kdx(mut self, kdx: f32) -> Self {
        self.kdx = Some(Frequency::new::<hertz>(kdx));
        self
    }

    pub fn ky(mut self, ky: f32) -> Self {
        self.ky = Some(GainType {
            value: ky,
            dimension: PhantomData,
            units: PhantomData,
        });
        self
    }

    pub fn kdy(mut self, kdy: f32) -> Self {
        self.kdy = Some(Frequency::new::<hertz>(kdy));
        self
    }

    pub fn valid_control_lower_bound(mut self, xi_threshold: Velocity) -> Self {
        self.xi_threshold = Some(xi_threshold);
        self
    }

    pub fn translation_controller(mut self, translation_controller: TC) -> Self
    where
        TC: TranslationController,
    {
        self.translation_controller = Some(translation_controller);
        self
    }

    pub fn rotation_controller(mut self, rotation_controller: RC) -> Self
    where
        RC: RotationController,
    {
        self.rotation_controller = Some(rotation_controller);
        self
    }

    pub fn left_motor(mut self, left_motor: LM) -> Self
    where
        LM: Motor,
    {
        self.left_motor = Some(left_motor);
        self
    }

    pub fn right_motor(mut self, right_motor: RM) -> Self
    where
        RM: Motor,
    {
        self.right_motor = Some(right_motor);
        self
    }

    pub fn period(mut self, period: Time) -> Self {
        self.period = Some(period);
        self
    }

    pub fn initial_velocity(mut self, xi: Velocity) -> Self {
        self.xi = Some(xi);
        self
    }

    pub fn fail_safe_distance(mut self, fail_safe_distance: Length) -> Self {
        self.fail_safe_distance = Some(fail_safe_distance);
        self
    }

    pub fn low_zeta(mut self, zeta: f32) -> Self {
        self.zeta = Some(zeta);
        self
    }

    pub fn low_b(mut self, b: f32) -> Self {
        self.b = Some(BType {
            value: b,
            ..Default::default()
        });
        self
    }

    pub fn build(self) -> Result<Tracker<LM, RM, M, TC, RC>, RequiredFieldEmptyError> {
        Ok(Tracker {
            kx: ok_or(self.kx, "kx")?,
            kdx: ok_or(self.kdx, "kdx")?,
            ky: ok_or(self.ky, "ky")?,
            kdy: ok_or(self.kdy, "kdy")?,
            xi_threshold: ok_or(self.xi_threshold, "xi_threshold")?,
            translation_controller: ok_or(self.translation_controller, "translation_controller")?,
            rotation_controller: ok_or(self.rotation_controller, "rotation_controller")?,
            left_motor: ok_or(self.left_motor, "left_motor")?,
            right_motor: ok_or(self.right_motor, "right_motor")?,
            period: ok_or(self.period, "period")?,
            xi: self.xi.expect("Should never None"),
            fail_safe_distance: ok_or(self.fail_safe_distance, "fail_safe_distance")?,
            zeta: ok_or(self.zeta, "zeta")?,
            b: ok_or(self.b, "b")?,
            _phantom: PhantomData,
        })
    }
}

impl<TC, RC, LM, RM> Default for TrackerBuilder<TC, RC, LM, RM, LibmMath> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use uom::si::{
        electric_potential::volt, length::meter, time::second, velocity::meter_per_second,
    };

    struct IMotor;

    impl Motor for IMotor {
        fn apply(&mut self, _voltage: ElectricPotential) {}
    }

    macro_rules! impl_controller {
        ($name: ident, $trait: ident: $t: ty, $dt: ty) => {
            struct $name {}

            impl $name {
                fn new() -> Self {
                    Self {}
                }
            }

            impl $trait for $name {
                fn init(&mut self) {}

                fn calculate(&mut self, _r: $t, _dr: $dt, _y: $t, _dy: $dt) -> ElectricPotential {
                    ElectricPotential::new::<volt>(1.0)
                }
            }
        };
    }

    impl_controller!(
        ITranslationController,
        TranslationController: Velocity,
        Acceleration
    );
    impl_controller!(
        IRotationController,
        RotationController: AngularVelocity,
        AngularAcceleration
    );

    fn build_tracker(
    ) -> Tracker<IMotor, IMotor, LibmMath, ITranslationController, IRotationController> {
        TrackerBuilder::default()
            .kx(1.0)
            .kdx(1.0)
            .ky(1.0)
            .kdy(1.0)
            .initial_velocity(Velocity::new::<meter_per_second>(0.0))
            .valid_control_lower_bound(Velocity::new::<meter_per_second>(0.001))
            .right_motor(IMotor)
            .left_motor(IMotor)
            .period(Time::new::<second>(0.001))
            .translation_controller(ITranslationController::new())
            .rotation_controller(IRotationController::new())
            .low_zeta(1.0)
            .low_b(1e-3)
            .fail_safe_distance(Length::new::<meter>(0.02))
            .build()
            .unwrap()
    }

    #[test]
    fn test_build() {
        let _tracker = build_tracker();
    }
}
