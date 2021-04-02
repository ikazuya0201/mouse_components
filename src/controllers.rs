//! Implementations of [Controller](crate::tracker::Controller) and their builders.

use core::ops::{Add, Div, Mul, Sub};

use uom::si::{
    f32::{
        Acceleration, AngularAcceleration, AngularVelocity, ElectricPotential, Length, Ratio, Time,
        Velocity,
    },
    Quantity,
};

use crate::impl_setter;
use crate::tracker::Controller as IController;
use crate::utils::builder::{ok_or, BuilderResult};

struct Controller<T>
where
    T: Div<Time>,
    <T as Div<Time>>::Output: Div<ElectricPotential> + Div<Time>,
    ElectricPotential: Div<T>
        + Div<<T as Div<Time>>::Output>
        + Div<<<T as Div<Time>>::Output as Div<Time>>::Output>,
{
    error_sum: T,
    model_gain: <<T as Div<Time>>::Output as Div<ElectricPotential>>::Output,
    time_constant: Time,
    kp: <ElectricPotential as Div<<T as Div<Time>>::Output>>::Output,
    ki: <ElectricPotential as Div<T>>::Output,
    kd: <ElectricPotential as Div<<<T as Div<Time>>::Output as Div<Time>>::Output>>::Output,
    period: Time,
}

impl<T> Controller<T>
where
    T: Div<Time> + Copy + Add<T, Output = T>,
    <T as Div<Time>>::Output: Div<Time>
        + Copy
        + Add<<T as Div<Time>>::Output, Output = <T as Div<Time>>::Output>
        + Sub<<T as Div<Time>>::Output, Output = <T as Div<Time>>::Output>
        + Div<ElectricPotential>
        + Div<
            <<T as Div<Time>>::Output as Div<ElectricPotential>>::Output,
            Output = ElectricPotential,
        > + Mul<Time, Output = T>,
    <<T as Div<Time>>::Output as Div<Time>>::Output: Mul<Time, Output = <T as Div<Time>>::Output>
        + Sub<
            <<T as Div<Time>>::Output as Div<Time>>::Output,
            Output = <<T as Div<Time>>::Output as Div<Time>>::Output,
        > + Copy,
    ElectricPotential: Div<T>
        + Div<<T as Div<Time>>::Output>
        + Div<<<T as Div<Time>>::Output as Div<Time>>::Output>,
    <ElectricPotential as Div<T>>::Output: Copy + Mul<T, Output = ElectricPotential>,
    <ElectricPotential as Div<<T as Div<Time>>::Output>>::Output:
        Copy + Mul<<T as Div<Time>>::Output, Output = ElectricPotential>,
    <<T as Div<Time>>::Output as Div<ElectricPotential>>::Output: Copy,
    <ElectricPotential as Div<<<T as Div<Time>>::Output as Div<Time>>::Output>>::Output:
        Copy + Mul<<<T as Div<Time>>::Output as Div<Time>>::Output, Output = ElectricPotential>,
{
    fn calculate(
        &mut self,
        r: <T as Div<Time>>::Output,
        dr: <<T as Div<Time>>::Output as Div<Time>>::Output,
        y: <T as Div<Time>>::Output,
        dy: <<T as Div<Time>>::Output as Div<Time>>::Output,
    ) -> ElectricPotential {
        let vol_f = (dr * self.time_constant + r) / self.model_gain;
        let vol_p = self.kp * (r - y);
        let vol_i = self.ki * self.error_sum;
        let vol_d = self.kd * (dr - dy);
        self.error_sum = self.error_sum + (r - y) * self.period;
        vol_f + vol_p + vol_i + vol_d
    }
}

macro_rules! impl_controller {
    ($controller: ident, $builder: ident, $dt: ty, $ddt: ty) => {
        impl $controller {
            pub fn new(
                model_gain: f32,
                model_time_constant: Time,
                kp: f32,
                ki: f32,
                kd: f32,
                period: Time,
            ) -> Self {
                Self(Controller {
                    error_sum: Default::default(),
                    model_gain: Quantity {
                        value: model_gain,
                        ..Default::default()
                    },
                    time_constant: model_time_constant,
                    kp: Quantity {
                        value: kp,
                        ..Default::default()
                    },
                    ki: Quantity {
                        value: ki,
                        ..Default::default()
                    },
                    kd: Quantity {
                        value: kd,
                        ..Default::default()
                    },
                    period,
                })
            }
        }

        impl IController<$dt, $ddt> for $controller {
            fn calculate(&mut self, r: $dt, dr: $ddt, y: $dt, dy: $ddt) -> ElectricPotential {
                self.0.calculate(r.into(), dr.into(), y.into(), dy.into())
            }
        }

        pub struct $builder {
            model_gain: Option<f32>,
            model_time_constant: Option<Time>,
            kp: Option<f32>,
            ki: Option<f32>,
            kd: Option<f32>,
            period: Option<Time>,
        }

        impl $builder {
            pub fn new() -> Self {
                Self {
                    model_gain: None,
                    model_time_constant: None,
                    kp: None,
                    ki: None,
                    kd: None,
                    period: None,
                }
            }

            impl_setter!(model_gain: f32);
            impl_setter!(model_time_constant: Time);
            impl_setter!(kp: f32);
            impl_setter!(ki: f32);
            impl_setter!(kd: f32);
            impl_setter!(period: Time);

            pub fn build(&mut self) -> BuilderResult<$controller> {
                macro_rules! get {
                    ($name: ident) => {
                        ok_or(self.$name, core::stringify!($name))?
                    };
                }
                Ok($controller::new(
                    get!(model_gain),
                    get!(model_time_constant),
                    get!(kp),
                    get!(ki),
                    get!(kd),
                    get!(period),
                ))
            }
        }
    };
}

pub struct TranslationalController(Controller<Length>);
impl_controller!(
    TranslationalController,
    TranslationalControllerBuilder,
    Velocity,
    Acceleration
);

pub struct RotationalController(Controller<Ratio>);
impl_controller!(
    RotationalController,
    RotationalControllerBuilder,
    AngularVelocity,
    AngularAcceleration
);

#[cfg(test)]
mod tests {
    use super::*;
    use typenum::consts::*;
    use uom::si::{time::second, Quantity, ISQ, SI};

    fn build_translation_controller() -> TranslationalController {
        TranslationalControllerBuilder::new()
            .period(Time::new::<second>(0.001))
            .model_gain(1.0)
            .model_time_constant(Time::new::<second>(0.01))
            .kp(1.0)
            .ki(0.1)
            .kd(0.2)
            .build()
            .unwrap()
    }

    #[test]
    fn test_build() {
        let _controller = build_translation_controller();
    }

    use uom::si::{
        acceleration::meter_per_second_squared, angular_acceleration::radian_per_second_squared,
        angular_velocity::radian_per_second, velocity::meter_per_second,
    };

    macro_rules! impl_model_simulator {
        ($mod: ident: $builder: ident, $gain: ident, $t: ty, $dt: ty, $tunit: ty, $dtunit: ty) => {
            mod $mod {
                use super::*;
                struct ModelSimulator {
                    gain: $gain,
                    time_constant: Time,
                    period: Time,
                    max_vol: ElectricPotential,
                    prev_vel: $t,
                    prev_accel: $dt,
                    prev_vol: ElectricPotential,
                }

                impl ModelSimulator {
                    fn new(
                        gain: f32,
                        time_constant: f32,
                        period: f32,
                        max_vol: ElectricPotential,
                    ) -> Self {
                        Self {
                            gain: $gain {
                                value: gain,
                                ..Default::default()
                            },
                            time_constant: Time::new::<second>(time_constant),
                            period: Time::new::<second>(period),
                            prev_vel: Default::default(),
                            prev_accel: Default::default(),
                            prev_vol: Default::default(),
                            max_vol,
                        }
                    }

                    fn step(&mut self, vol: ElectricPotential) -> ($t, $dt) {
                        let vol = if vol > self.max_vol {
                            self.max_vol
                        } else if vol < -self.max_vol {
                            -self.max_vol
                        } else {
                            vol
                        };
                        let vel = <$t>::from(
                            (self.time_constant * self.prev_vel + self.gain * self.period * vol)
                                / (self.period + self.time_constant),
                        );
                        let accel = <$dt>::from(
                            (self.time_constant * self.prev_accel
                                + self.gain * (vol - self.prev_vol))
                                / (self.period + self.time_constant),
                        );
                        self.prev_vel = vel;
                        self.prev_accel = accel;
                        self.prev_vol = vol;
                        (vel, accel)
                    }
                }

                #[test]
                fn test_controller() {
                    use uom::si::electric_potential::volt;

                    let period = 0.001;
                    let time_constant = 0.3694;
                    let gain = 3.3;
                    let mut controller = $builder::new()
                        .period(Time::new::<second>(period))
                        .model_gain(gain)
                        .model_time_constant(Time::new::<second>(time_constant))
                        .kp(0.9)
                        .ki(0.05)
                        .kd(0.01)
                        .build()
                        .unwrap();
                    let mut simulator = ModelSimulator::new(
                        gain,
                        time_constant,
                        period,
                        ElectricPotential::new::<volt>(3.3),
                    );

                    let target_vel = <$t>::new::<$tunit>(1.0);
                    let target_accel = <$dt>::new::<$dtunit>(0.0);
                    let mut cur_vel = Default::default();
                    let mut cur_accel = Default::default();
                    for _ in 0..1000 {
                        let vol =
                            controller.calculate(target_vel, target_accel, cur_vel, cur_accel);
                        let (vel, accel) = simulator.step(vol);
                        cur_vel = vel;
                        cur_accel = accel;
                    }
                }
            }
        };
    }

    type TransGain = Quantity<ISQ<N1, N1, P2, P1, Z0, Z0, Z0>, SI<f32>, f32>;
    type RotGain = Quantity<ISQ<N2, N1, P2, P1, Z0, Z0, Z0>, SI<f32>, f32>;
    impl_model_simulator!(
        trans_controller: TranslationalControllerBuilder,
        TransGain,
        Velocity,
        Acceleration,
        meter_per_second,
        meter_per_second_squared
    );
    impl_model_simulator!(
        rot_controller: RotationalControllerBuilder,
        RotGain,
        AngularVelocity,
        AngularAcceleration,
        radian_per_second,
        radian_per_second_squared
    );
}
