use crate::tracker;
use uom::si::{
    acceleration::meter_per_second_squared,
    angle::radian,
    angular_acceleration::radian_per_second_squared,
    angular_velocity::radian_per_second,
    electric_potential::volt,
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularVelocity, ElectricPotential, Length, Time,
        Velocity,
    },
    length::meter,
    velocity::meter_per_second,
};

macro_rules! impl_controller {
    (
        $controller_name: ident,
        $trait: ident,
        $builder_name: ident: $t: ty,
        $dt: ty,
        $ddt: ty,
        $tunit: ty,
        $dtunit: ty,
        $ddtunit: ty,
    ) => {
        ///the model is a first-order delay system
        pub struct $controller_name {
            error_sum: $t,
            model_gain: f32,
            time_constant: Time,
            kp: f32,
            ki: f32,
            kd: f32,
            period: Time,
        }

        impl $controller_name {
            pub fn new(
                kp: f32,
                ki: f32,
                kd: f32,
                period: Time,
                model_gain: f32,
                model_time_constant: Time,
            ) -> Self {
                Self {
                    error_sum: Default::default(),
                    model_gain,
                    time_constant: model_time_constant,
                    kp,
                    ki,
                    kd,
                    period,
                }
            }
        }

        impl tracker::$trait for $controller_name {
            fn init(&mut self) {
                self.error_sum = Default::default();
            }

            fn calculate(&mut self, r: $dt, dr: $ddt, y: $dt, dy: $ddt) -> ElectricPotential {
                let vol_f =
                    (<$dt>::from(dr * self.time_constant) + r).get::<$dtunit>() / self.model_gain;
                let vol_p = self.kp * (r - y).get::<$dtunit>();
                let vol_i = self.ki * self.error_sum.get::<$tunit>();
                let vol_d = self.kd * (dr - dy).get::<$ddtunit>();
                self.error_sum = self.error_sum + <$t>::from((r - y) * self.period);
                ElectricPotential::new::<volt>(vol_f + vol_p + vol_i + vol_d)
            }
        }

        pub struct $builder_name<KP, KI, KD, P, MG, MT> {
            kp: KP,
            ki: KI,
            kd: KD,
            period: P,
            model_gain: MG,
            model_time_constant: MT,
        }

        impl $builder_name<(), (), (), (), (), ()> {
            pub fn new() -> Self {
                Self {
                    kp: (),
                    ki: (),
                    kd: (),
                    period: (),
                    model_gain: (),
                    model_time_constant: (),
                }
            }
        }

        impl $builder_name<f32, f32, f32, Time, f32, Time> {
            pub fn build(self) -> $controller_name {
                $controller_name {
                    error_sum: Default::default(),
                    model_gain: self.model_gain,
                    time_constant: self.model_time_constant,
                    kp: self.kp,
                    ki: self.ki,
                    kd: self.kd,
                    period: self.period,
                }
            }
        }

        impl<KI, KD, P, MG, MT> $builder_name<(), KI, KD, P, MG, MT> {
            pub fn kp(self, kp: f32) -> $builder_name<f32, KI, KD, P, MG, MT> {
                $builder_name {
                    kp,
                    ki: self.ki,
                    kd: self.kd,
                    period: self.period,
                    model_gain: self.model_gain,
                    model_time_constant: self.model_time_constant,
                }
            }
        }

        impl<KP, KD, P, MG, MT> $builder_name<KP, (), KD, P, MG, MT> {
            pub fn ki(self, ki: f32) -> $builder_name<KP, f32, KD, P, MG, MT> {
                $builder_name {
                    kp: self.kp,
                    ki,
                    kd: self.kd,
                    period: self.period,
                    model_gain: self.model_gain,
                    model_time_constant: self.model_time_constant,
                }
            }
        }

        impl<KP, KI, P, MG, MT> $builder_name<KP, KI, (), P, MG, MT> {
            pub fn kd(self, kd: f32) -> $builder_name<KP, KI, f32, P, MG, MT> {
                $builder_name {
                    kp: self.kp,
                    ki: self.ki,
                    kd,
                    period: self.period,
                    model_gain: self.model_gain,
                    model_time_constant: self.model_time_constant,
                }
            }
        }

        impl<KP, KI, KD, MG, MT> $builder_name<KP, KI, KD, (), MG, MT> {
            pub fn period(self, period: Time) -> $builder_name<KP, KI, KD, Time, MG, MT> {
                $builder_name {
                    kp: self.kp,
                    ki: self.ki,
                    kd: self.kd,
                    period,
                    model_gain: self.model_gain,
                    model_time_constant: self.model_time_constant,
                }
            }
        }

        impl<KP, KI, KD, P, MT> $builder_name<KP, KI, KD, P, (), MT> {
            pub fn model_gain(self, model_gain: f32) -> $builder_name<KP, KI, KD, P, f32, MT> {
                $builder_name {
                    kp: self.kp,
                    ki: self.ki,
                    kd: self.kd,
                    period: self.period,
                    model_gain,
                    model_time_constant: self.model_time_constant,
                }
            }
        }

        impl<KP, KI, KD, P, MG> $builder_name<KP, KI, KD, P, MG, ()> {
            pub fn model_time_constant(
                self,
                model_time_constant: Time,
            ) -> $builder_name<KP, KI, KD, P, MG, Time> {
                $builder_name {
                    kp: self.kp,
                    ki: self.ki,
                    kd: self.kd,
                    period: self.period,
                    model_gain: self.model_gain,
                    model_time_constant,
                }
            }
        }
    };
}

impl_controller!(
    TranslationController,
    TranslationController,
    TranslationControllerBuilder: Length,
    Velocity,
    Acceleration,
    meter,
    meter_per_second,
    meter_per_second_squared,
);

impl_controller!(
    RotationController,
    RotationController,
    RotationControllerBuilder: Angle,
    AngularVelocity,
    AngularAcceleration,
    radian,
    radian_per_second,
    radian_per_second_squared,
);

#[cfg(test)]
mod tests {
    use super::*;
    use typenum::consts::*;
    use uom::si::{time::second, Quantity, ISQ, SI};

    use crate::prelude::*;

    fn build_translation_controller() -> TranslationController {
        TranslationControllerBuilder::new()
            .period(Time::new::<second>(0.001))
            .model_gain(1.0)
            .model_time_constant(Time::new::<second>(0.01))
            .kp(1.0)
            .ki(0.1)
            .kd(0.2)
            .build()
    }

    #[test]
    fn test_build() {
        let _controller = build_translation_controller();
    }

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
                        .build();
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
                        dbg!(vel, accel);
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
        trans_controller: TranslationControllerBuilder,
        TransGain,
        Velocity,
        Acceleration,
        meter_per_second,
        meter_per_second_squared
    );
    impl_model_simulator!(
        rot_controller: RotationControllerBuilder,
        RotGain,
        AngularVelocity,
        AngularAcceleration,
        radian_per_second,
        radian_per_second_squared
    );
}
