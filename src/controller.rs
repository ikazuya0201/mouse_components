use crate::quantities::{
    acceleration::meter_per_second_squared,
    dimensionless::radian,
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularVelocity, Length, Time, Velocity, Voltage,
    },
    frequency::radian_per_second,
    length::meter,
    squared_frequency::radian_per_second_squared,
    velocity::meter_per_second,
    voltage::volt,
};
use crate::tracker;

macro_rules! impl_controller {
    (
        $controller_name: ident,
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

        impl tracker::Controller<$t> for $controller_name {
            fn init(&mut self) {
                self.error_sum = Default::default();
            }

            fn calculate(&mut self, r: $dt, dr: $ddt, y: $dt, dy: $ddt) -> Voltage {
                let vol_f = (dr * self.time_constant + r).get::<$dtunit>() / self.model_gain;
                let vol_p = self.kp * (r - y).get::<$dtunit>();
                let vol_i = self.ki * self.error_sum.get::<$tunit>();
                let vol_d = self.kd * (dr - dy).get::<$ddtunit>();
                self.error_sum = self.error_sum + (r - y) * self.period;
                Voltage::new::<volt>(vol_f + vol_p + vol_i + vol_d)
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
    TranslationControllerBuilder: Length,
    Velocity,
    Acceleration,
    meter,
    meter_per_second,
    meter_per_second_squared,
);

impl_controller!(
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
    use crate::quantities::time::second;

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
}
