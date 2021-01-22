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

use super::agents::Tracker as ITracker;
use super::trajectory_generator::{AngleTarget, MoveTarget, Target};
use crate::traits::Math;
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
    TC = crate::impls::TranslationController,
    RC = crate::impls::RotationController,
    L = NullLogger,
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
    #[allow(unused)]
    logger: L,
    zeta: f32,
    b: BType,
    _phantom: PhantomData<fn() -> M>,
}

impl<LM, RM, M, TC, RC, L> core::fmt::Debug for Tracker<LM, RM, M, TC, RC, L> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "Tracker{{ xi:{:?} }}", self.xi)
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct FailSafeError {
    state: State,
    target: MoveTarget,
}

impl<LM, RM, M, TC, RC, L> ITracker<State, Target> for Tracker<LM, RM, M, TC, RC, L>
where
    LM: Motor,
    RM: Motor,
    TC: TranslationController,
    RC: RotationController,
    L: Logger,
    M: Math,
{
    type Error = FailSafeError;

    fn stop(&mut self)
    where
        LM: Motor,
        RM: Motor,
    {
        self.left_motor.apply(Default::default());
        self.right_motor.apply(Default::default());
    }

    fn init(&mut self) {
        self.translation_controller.init();
        self.rotation_controller.init();
    }

    fn track(&mut self, state: &State, target: &Target) -> Result<(), Self::Error> {
        self.logger.log(&state, &target);
        let (left, right) = match target {
            Target::Moving(target) => self.track_move(state, target)?,
            Target::Spin(target) => self.track_spin(state, target),
        };
        self.left_motor.apply(left);
        self.right_motor.apply(right);
        Ok(())
    }
}

impl<LM, RM, M, TC, RC, L> Tracker<LM, RM, M, TC, RC, L>
where
    LM: Motor,
    RM: Motor,
    TC: TranslationController,
    RC: RotationController,
    L: Logger,
    M: Math,
{
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

pub struct TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B> {
    kx: KX,
    kdx: KDX,
    ky: KY,
    kdy: KDY,
    xi_threshold: XIT,
    translation_controller: TC,
    rotation_controller: RC,
    left_motor: LM,
    right_motor: RM,
    period: P,
    xi: XI,
    fail_safe_distance: FS,
    logger: L,
    zeta: Z,
    b: B,
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
{
    const DEFAULT_FAIL_SAFE_DISTANCE: Length = Length {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.02,
    };
}

impl TrackerBuilder<(), (), (), (), (), (), (), (), (), (), (), (), NullLogger, (), ()> {
    pub fn new() -> Self {
        Self {
            kx: (),
            kdx: (),
            ky: (),
            kdy: (),
            xi_threshold: (),
            translation_controller: (),
            rotation_controller: (),
            left_motor: (),
            right_motor: (),
            period: (),
            xi: (),
            fail_safe_distance: (),
            logger: NullLogger,
            zeta: (),
            b: (),
        }
    }
}

impl<TC, RC, LM, RM, L>
    TrackerBuilder<
        GainType,
        Frequency,
        GainType,
        Frequency,
        Velocity,
        TC,
        RC,
        LM,
        RM,
        Time,
        (),
        (),
        L,
        f32,
        BType,
    >
where
    LM: Motor,
    RM: Motor,
    TC: TranslationController,
    RC: RotationController,
    L: Logger,
{
    pub fn build<M>(self) -> Tracker<LM, RM, M, TC, RC, L>
    where
        M: Math,
    {
        Tracker {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: Default::default(),
            fail_safe_distance: Self::DEFAULT_FAIL_SAFE_DISTANCE,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
            _phantom: PhantomData,
        }
    }
}

impl<TC, RC, LM, RM, L>
    TrackerBuilder<
        GainType,
        Frequency,
        GainType,
        Frequency,
        Velocity,
        TC,
        RC,
        LM,
        RM,
        Time,
        (),
        Length,
        L,
        f32,
        BType,
    >
where
    LM: Motor,
    RM: Motor,
    TC: TranslationController,
    RC: RotationController,
    L: Logger,
{
    pub fn build<M>(self) -> Tracker<LM, RM, M, TC, RC, L>
    where
        M: Math,
    {
        Tracker {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: Default::default(),
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
            _phantom: PhantomData,
        }
    }
}

impl<TC, RC, LM, RM, L>
    TrackerBuilder<
        GainType,
        Frequency,
        GainType,
        Frequency,
        Velocity,
        TC,
        RC,
        LM,
        RM,
        Time,
        Velocity,
        (),
        L,
        f32,
        BType,
    >
where
    LM: Motor,
    RM: Motor,
    TC: TranslationController,
    RC: RotationController,
{
    pub fn build<M>(self) -> Tracker<LM, RM, M, TC, RC, L>
    where
        M: Math,
    {
        Tracker {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance: Self::DEFAULT_FAIL_SAFE_DISTANCE,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
            _phantom: PhantomData,
        }
    }
}

impl<TC, RC, LM, RM, L>
    TrackerBuilder<
        GainType,
        Frequency,
        GainType,
        Frequency,
        Velocity,
        TC,
        RC,
        LM,
        RM,
        Time,
        Velocity,
        Length,
        L,
        f32,
        BType,
    >
where
    LM: Motor,
    RM: Motor,
    TC: TranslationController,
    RC: RotationController,
    L: Logger,
{
    pub fn build<M>(self) -> Tracker<LM, RM, M, TC, RC, L>
    where
        M: Math,
    {
        Tracker {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
            _phantom: PhantomData,
        }
    }
}

impl<KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
    TrackerBuilder<(), KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
{
    pub fn kx(
        self,
        kx: f32,
    ) -> TrackerBuilder<GainType, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B> {
        TrackerBuilder {
            kx: GainType {
                value: kx,
                ..Default::default()
            },
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
        }
    }
}

impl<KX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
    TrackerBuilder<KX, (), KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
{
    pub fn kdx(
        self,
        kdx: f32,
    ) -> TrackerBuilder<KX, Frequency, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B> {
        TrackerBuilder {
            kx: self.kx,
            kdx: Frequency::new::<hertz>(kdx),
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
        }
    }
}

impl<KX, KDX, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
    TrackerBuilder<KX, KDX, (), KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
{
    pub fn ky(
        self,
        ky: f32,
    ) -> TrackerBuilder<KX, KDX, GainType, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: GainType {
                value: ky,
                ..Default::default()
            },
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
        }
    }
}

impl<KX, KDX, KY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
    TrackerBuilder<KX, KDX, KY, (), XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
{
    pub fn kdy(
        self,
        kdy: f32,
    ) -> TrackerBuilder<KX, KDX, KY, Frequency, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: Frequency::new::<hertz>(kdy),
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
        }
    }
}

impl<KX, KDX, KY, KDY, TC, RC, LM, RM, P, XI, FS, L, Z, B>
    TrackerBuilder<KX, KDX, KY, KDY, (), TC, RC, LM, RM, P, XI, FS, L, Z, B>
{
    pub fn valid_control_lower_bound(
        self,
        xi_threshold: Velocity,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, Velocity, TC, RC, LM, RM, P, XI, FS, L, Z, B> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, RC, LM, RM, P, XI, FS, L, Z, B>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, (), RC, LM, RM, P, XI, FS, L, Z, B>
{
    pub fn translation_controller<TC>(
        self,
        translation_controller: TC,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
    where
        TC: TranslationController,
    {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, LM, RM, P, XI, FS, L, Z, B>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, (), LM, RM, P, XI, FS, L, Z, B>
{
    pub fn rotation_controller<RC>(
        self,
        rotation_controller: RC,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
    where
        RC: RotationController,
    {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, RM, P, XI, FS, L, Z, B>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, (), RM, P, XI, FS, L, Z, B>
{
    pub fn left_motor<LM>(
        self,
        left_motor: LM,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
    where
        LM: Motor,
    {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, LM, P, XI, FS, L, Z, B>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, (), P, XI, FS, L, Z, B>
{
    pub fn right_motor<RM>(
        self,
        right_motor: RM,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
    where
        RM: Motor,
    {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, XI, FS, L, Z, B>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, (), XI, FS, L, Z, B>
{
    pub fn period(
        self,
        period: Time,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, Time, XI, FS, L, Z, B> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period,
            xi: self.xi,
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, FS, L, Z, B>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, (), FS, L, Z, B>
{
    pub fn initial_velocity(
        self,
        xi: Velocity,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, Velocity, FS, L, Z, B> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi,
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, L, Z, B>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, (), L, Z, B>
{
    pub fn fail_safe_distance(
        self,
        fail_safe_distance: Length,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, Length, L, Z, B> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: self.b,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, Z, B>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, NullLogger, Z, B>
{
    pub fn logger<L>(
        self,
        logger: L,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, B>
    where
        L: Logger,
    {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance: self.fail_safe_distance,
            logger,
            zeta: self.zeta,
            b: self.b,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, B>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, (), B>
{
    pub fn low_zeta(
        self,
        zeta: f32,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, f32, B> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta,
            b: self.b,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, ()>
{
    pub fn low_b(
        self,
        b: f32,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, FS, L, Z, BType> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
            fail_safe_distance: self.fail_safe_distance,
            logger: self.logger,
            zeta: self.zeta,
            b: BType {
                value: b,
                ..Default::default()
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::math::MathFake;
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

    fn build_tracker<L>(
        logger: L,
    ) -> Tracker<IMotor, IMotor, MathFake, ITranslationController, IRotationController, L>
    where
        L: Logger,
    {
        TrackerBuilder::new()
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
            .logger(logger)
            .build()
    }

    #[test]
    fn test_build() {
        let _tracker = build_tracker(NullLogger);
    }

    mod log_tests {
        use super::*;
        use std::cell::RefCell;
        use std::fmt::Write;
        use std::rc::Rc;

        struct ILogger {
            raw: Rc<RefCell<String>>,
        }

        impl ILogger {
            fn new(raw: Rc<RefCell<String>>) -> Self {
                Self { raw }
            }
        }

        impl Logger for ILogger {
            fn log(&self, state: &State, target: &Target) {
                write!(self.raw.borrow_mut(), "{:?},{:?}", state, target).unwrap();
            }
        }

        #[test]
        fn test_log() {
            use crate::prelude::*;

            let log = Rc::new(RefCell::new(String::new()));
            let state = State::default();
            let target = Target::Moving(MoveTarget::default());
            let mut tracker = build_tracker(ILogger::new(Rc::clone(&log)));
            tracker.track(&state, &target).unwrap();
            assert_eq!(log.borrow().as_ref(), format!("{:?},{:?}", state, target));
        }
    }
}
