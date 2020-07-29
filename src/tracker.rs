mod motor;
mod state;

use core::ops::Div;

use quantities::{
    Acceleration, Angle, AngularAcceleration, AngularSpeed, Distance, Frequency, Speed,
    SquaredFrequency, Time, TimeDifferentiable, Voltage,
};

use super::agent;
use super::trajectory_generator::Target;
use crate::{ddt, dt};
pub use motor::Motor;
pub use state::{State, SubState};

pub trait Controller<T>
where
    T: TimeDifferentiable,
    dt!(T): TimeDifferentiable,
{
    fn calculate(&mut self, r: dt!(T), dr: ddt!(T), y: dt!(T), dy: ddt!(T)) -> Voltage;
}

pub struct Tracker<LM, RM, TC, RC> {
    kx: Frequency,
    kdx: SquaredFrequency,
    ky: Frequency,
    kdy: SquaredFrequency,
    kn_kx: Frequency,
    kn_ky: f32,
    kn_ktheta: f32,
    xi: Speed,
    period: Time,
    xi_threshold: Speed,
    translation_controller: TC,
    rotation_controller: RC,
    left_motor: LM,
    right_motor: RM,
}

impl<LM, RM, TC, RC> agent::Tracker<State, Target> for Tracker<LM, RM, TC, RC>
where
    LM: Motor,
    RM: Motor,
    TC: Controller<Distance>,
    RC: Controller<Angle>,
{
    fn track(&mut self, state: State, target: Target) {
        let (left, right) = self.track_move(state, target);
        self.left_motor.apply(left);
        self.right_motor.apply(right);
    }
}

impl<LM, RM, TC, RC> Tracker<LM, RM, TC, RC>
where
    TC: Controller<Distance>,
    RC: Controller<Angle>,
{
    #[cfg(feature = "debug")]
    pub fn stop(&mut self)
    where
        LM: Motor,
        RM: Motor,
    {
        self.left_motor.apply(Default::default());
        self.right_motor.apply(Default::default());
    }

    fn track_move(&mut self, state: State, target: Target) -> (Voltage, Voltage) {
        let cos_theta = state.theta.x.cos();
        let sin_theta = state.theta.x.sin();

        let vv = state.x.v * cos_theta + state.y.v * sin_theta;
        let va = state.x.a * cos_theta + state.y.a * sin_theta;

        //calculate control input for (x,y)
        let ux =
            target.x.a + self.kx * (target.x.v - state.x.v) + self.kdx * (target.x.x - state.x.x);
        let uy =
            target.y.a + self.ky * (target.y.v - state.y.v) + self.kdy * (target.y.x - state.y.x);
        let dux =
            target.x.j + self.kx * (target.x.a - state.x.a) + self.kdx * (target.x.v - state.x.v);
        let duy =
            target.y.j + self.ky * (target.y.a - state.y.a) + self.kdy * (target.y.v - state.y.v);

        let dxi = ux * cos_theta + uy * sin_theta;
        let (uv, uw, duv, duw) = if self.xi > self.xi_threshold {
            let uv = self.xi;
            let uw = (uy * cos_theta - ux * sin_theta) / self.xi;
            let duv = dxi;
            let duw = -(2.0 * dxi * uw + dux * sin_theta - duy * cos_theta) / self.xi;
            (uv, uw, duv, duw)
        } else {
            let vx_raw = target.x.v.as_meter_per_second();
            let vy_raw = target.y.v.as_meter_per_second();
            let vr = Speed::from_meter_per_second(libm::sqrtf(vx_raw * vx_raw + vy_raw * vy_raw));

            let ax_raw = target.x.a.as_meter_per_second_squared();
            let ay_raw = target.y.a.as_meter_per_second_squared();
            let ar = Acceleration::from_meter_per_second_squared(libm::sqrtf(
                ax_raw * ax_raw + ay_raw * ay_raw,
            ));

            let theta_e = target.theta.x - state.theta.x;
            let x_e = target.x.x - state.x.x;
            let y_e = target.y.x - state.y.x;

            let w_e = target.theta.v - state.theta.v;
            let vx_e = target.x.v - state.x.v;
            let vy_e = target.y.v - state.y.v;

            let sin_th_e = theta_e.sin();
            let cos_th_e = theta_e.cos();

            let uv = vr * cos_th_e + self.kn_kx * x_e;
            let uw = target.theta.v
                + AngularSpeed::from_radian_per_second(
                    vr.as_meter_per_second()
                        * (self.kn_ky * y_e.as_meters() + self.kn_ktheta * sin_th_e),
                );

            let duv = ar * cos_th_e - vr * sin_th_e * w_e + self.kn_kx * vx_e;
            let duw = target.theta.a
                + AngularAcceleration::from_radian_per_second_squared(
                    ar.as_meter_per_second_squared()
                        * (self.kn_ky * y_e.as_meters() + self.kn_ktheta * sin_th_e)
                        + vr.as_meter_per_second()
                            * (self.kn_ky * vy_e.as_meter_per_second()
                                + self.kn_ktheta * cos_th_e * w_e.as_radian_per_second()),
                );

            (uv, uw, duv, duw)
        };

        self.xi += self.period * dxi;
        //calculate motor voltage
        let vol_v = self.translation_controller.calculate(uv, duv, vv, va);
        let vol_w = self
            .rotation_controller
            .calculate(uw, duw, state.theta.v, state.theta.a);
        (vol_v - vol_w, vol_v + vol_w)
    }
}

pub struct TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT> {
    kx: KX,
    kdx: KDX,
    ky: KY,
    kdy: KDY,
    kn_kx: KKX,
    kn_ky: KKY,
    kn_ktheta: KKT,
    xi_threshold: XIT,
    translation_controller: TC,
    rotation_controller: RC,
    left_motor: LM,
    right_motor: RM,
    period: P,
    xi: XI,
}

impl TrackerBuilder<(), (), (), (), (), (), (), (), (), (), (), (), (), ()> {
    pub fn new() -> Self {
        Self {
            kx: (),
            kdx: (),
            ky: (),
            kdy: (),
            kn_kx: (),
            kn_ky: (),
            kn_ktheta: (),
            xi_threshold: (),
            translation_controller: (),
            rotation_controller: (),
            left_motor: (),
            right_motor: (),
            period: (),
            xi: (),
        }
    }
}

impl<TC, RC, LM, RM>
    TrackerBuilder<f32, f32, f32, f32, Speed, TC, RC, LM, RM, Time, (), f32, f32, f32>
where
    LM: Motor,
    RM: Motor,
    TC: Controller<Distance>,
    RC: Controller<Angle>,
{
    pub fn build(self) -> Tracker<LM, RM, TC, RC> {
        Tracker {
            kx: Frequency::from_hertz(self.kx),
            kdx: SquaredFrequency::from_squared_hertz(self.kdx),
            ky: Frequency::from_hertz(self.ky),
            kdy: SquaredFrequency::from_squared_hertz(self.kdy),
            kn_kx: Frequency::from_hertz(self.kn_kx),
            kn_ky: self.kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: Default::default(),
        }
    }
}

impl<TC, RC, LM, RM>
    TrackerBuilder<f32, f32, f32, f32, Speed, TC, RC, LM, RM, Time, Speed, f32, f32, f32>
where
    LM: Motor,
    RM: Motor,
    TC: Controller<Distance>,
    RC: Controller<Angle>,
{
    pub fn build(self) -> Tracker<LM, RM, TC, RC> {
        Tracker {
            kx: Frequency::from_hertz(self.kx),
            kdx: SquaredFrequency::from_squared_hertz(self.kdx),
            ky: Frequency::from_hertz(self.ky),
            kdy: SquaredFrequency::from_squared_hertz(self.kdy),
            kn_kx: Frequency::from_hertz(self.kn_kx),
            kn_ky: self.kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
        }
    }
}

impl<KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT>
    TrackerBuilder<(), KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT>
{
    pub fn kx(
        self,
        kx: f32,
    ) -> TrackerBuilder<f32, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT> {
        TrackerBuilder {
            kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            kn_kx: self.kn_kx,
            kn_ky: self.kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
        }
    }
}

impl<KX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT>
    TrackerBuilder<KX, (), KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT>
{
    pub fn kdx(
        self,
        kdx: f32,
    ) -> TrackerBuilder<KX, f32, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT> {
        TrackerBuilder {
            kx: self.kx,
            kdx,
            ky: self.ky,
            kdy: self.kdy,
            kn_kx: self.kn_kx,
            kn_ky: self.kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
        }
    }
}

impl<KX, KDX, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT>
    TrackerBuilder<KX, KDX, (), KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT>
{
    pub fn ky(
        self,
        ky: f32,
    ) -> TrackerBuilder<KX, KDX, f32, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky,
            kdy: self.kdy,
            kn_kx: self.kn_kx,
            kn_ky: self.kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
        }
    }
}

impl<KX, KDX, KY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT>
    TrackerBuilder<KX, KDX, KY, (), XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT>
{
    pub fn kdy(
        self,
        kdy: f32,
    ) -> TrackerBuilder<KX, KDX, KY, f32, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy,
            kn_kx: self.kn_kx,
            kn_ky: self.kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
        }
    }
}

impl<KX, KDX, KY, KDY, TC, RC, LM, RM, P, XI, KKX, KKY, KKT>
    TrackerBuilder<KX, KDX, KY, KDY, (), TC, RC, LM, RM, P, XI, KKX, KKY, KKT>
{
    pub fn valid_control_lower_bound(
        self,
        xi_threshold: Speed,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, Speed, TC, RC, LM, RM, P, XI, KKX, KKY, KKT> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            kn_kx: self.kn_kx,
            kn_ky: self.kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, RC, LM, RM, P, XI, KKX, KKY, KKT>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, (), RC, LM, RM, P, XI, KKX, KKY, KKT>
{
    pub fn translation_controller<TC>(
        self,
        translation_controller: TC,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT>
    where
        TC: Controller<Distance>,
    {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            kn_kx: self.kn_kx,
            kn_ky: self.kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, LM, RM, P, XI, KKX, KKY, KKT>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, (), LM, RM, P, XI, KKX, KKY, KKT>
{
    pub fn rotation_controller<RC>(
        self,
        rotation_controller: RC,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT>
    where
        RC: Controller<Angle>,
    {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            kn_kx: self.kn_kx,
            kn_ky: self.kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, RM, P, XI, KKX, KKY, KKT>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, (), RM, P, XI, KKX, KKY, KKT>
{
    pub fn left_motor<LM>(
        self,
        left_motor: LM,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT>
    where
        LM: Motor,
    {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            kn_kx: self.kn_kx,
            kn_ky: self.kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, LM, P, XI, KKX, KKY, KKT>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, (), P, XI, KKX, KKY, KKT>
{
    pub fn right_motor<RM>(
        self,
        right_motor: RM,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, KKT>
    where
        RM: Motor,
    {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            kn_kx: self.kn_kx,
            kn_ky: self.kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor,
            period: self.period,
            xi: self.xi,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, XI, KKX, KKY, KKT>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, (), XI, KKX, KKY, KKT>
{
    pub fn period(
        self,
        period: Time,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, Time, XI, KKX, KKY, KKT> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            kn_kx: self.kn_kx,
            kn_ky: self.kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period,
            xi: self.xi,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, KKX, KKY, KKT>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, (), KKX, KKY, KKT>
{
    pub fn initial_speed(
        self,
        xi: Speed,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, Speed, KKX, KKY, KKT> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            kn_kx: self.kn_kx,
            kn_ky: self.kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKY, KKT>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, (), KKY, KKT>
{
    pub fn kaneyama_kx(
        self,
        kn_kx: f32,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, f32, KKY, KKT> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            kn_kx,
            kn_ky: self.kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKT>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, (), KKT>
{
    pub fn kaneyama_ky(
        self,
        kn_ky: f32,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, f32, KKT> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            kn_kx: self.kn_kx,
            kn_ky,
            kn_ktheta: self.kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
        }
    }
}

impl<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY>
    TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, ()>
{
    pub fn kaneyama_ktheta(
        self,
        kn_ktheta: f32,
    ) -> TrackerBuilder<KX, KDX, KY, KDY, XIT, TC, RC, LM, RM, P, XI, KKX, KKY, f32> {
        TrackerBuilder {
            kx: self.kx,
            kdx: self.kdx,
            ky: self.ky,
            kdy: self.kdy,
            kn_kx: self.kn_kx,
            kn_ky: self.kn_ky,
            kn_ktheta,
            xi_threshold: self.xi_threshold,
            translation_controller: self.translation_controller,
            rotation_controller: self.rotation_controller,
            left_motor: self.left_motor,
            right_motor: self.right_motor,
            period: self.period,
            xi: self.xi,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::marker::PhantomData;

    struct IMotor;

    impl Motor for IMotor {
        fn apply(&mut self, _voltage: Voltage) {}
    }

    struct IController<T> {
        _phantom: PhantomData<fn() -> T>,
    }

    impl<T> IController<T> {
        fn new() -> Self {
            Self {
                _phantom: PhantomData,
            }
        }
    }

    impl<T> Controller<T> for IController<T>
    where
        T: TimeDifferentiable,
        dt!(T): TimeDifferentiable,
    {
        fn calculate(&mut self, _r: dt!(T), _dr: ddt!(T), _y: dt!(T), _dy: ddt!(T)) -> Voltage {
            Voltage::from_volts(1.0)
        }
    }

    fn build_tracker() -> Tracker<IMotor, IMotor, IController<Distance>, IController<Angle>> {
        TrackerBuilder::new()
            .kx(1.0)
            .kdx(1.0)
            .ky(1.0)
            .kdy(1.0)
            .initial_speed(Speed::from_meter_per_second(0.0))
            .valid_control_lower_bound(Speed::from_meter_per_second(0.001))
            .right_motor(IMotor)
            .left_motor(IMotor)
            .period(Time::from_seconds(0.001))
            .translation_controller(IController::<Distance>::new())
            .rotation_controller(IController::<Angle>::new())
            .kaneyama_kx(1.0)
            .kaneyama_ky(1.0)
            .kaneyama_ktheta(1.0)
            .build()
    }

    #[test]
    fn test_build() {
        let _tracker = build_tracker();
    }
}
