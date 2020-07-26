use core::marker::PhantomData;
use core::ops::{Div, Mul};

use quantities::{Fraction, Quantity, Time, TimeDifferentiable, Voltage};

use crate::tracker;
use crate::{ddt, dt};

///the model is a first-order delay system
pub struct Controller<T>
where
    T: TimeDifferentiable,
    dt!(T): TimeDifferentiable,
{
    error_sum: T,
    rev_gain: Fraction<Voltage, dt!(T)>,
    time_constant: Time,
    kp: Fraction<Voltage, dt!(T)>,
    ki: Fraction<Voltage, T>,
    kd: Fraction<Voltage, ddt!(T)>,
    period: Time,
}

impl<T> Controller<T>
where
    T: TimeDifferentiable,
    dt!(T): TimeDifferentiable,
{
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
            rev_gain: Fraction::from_raw(1.0 / model_gain),
            time_constant: model_time_constant,
            kp: Fraction::from_raw(kp),
            ki: Fraction::from_raw(ki),
            kd: Fraction::from_raw(kd),
            period,
        }
    }
}

impl<T> tracker::Controller<T> for Controller<T>
where
    T: TimeDifferentiable,
    dt!(T): TimeDifferentiable + Mul<Time, Output = T>,
    ddt!(T): Quantity + Mul<Time, Output = dt!(T)>,
    f32: From<T> + From<dt!(T)> + From<ddt!(T)>,
{
    fn calculate(&mut self, r: dt!(T), dr: ddt!(T), y: dt!(T), dy: ddt!(T)) -> Voltage {
        let vol_f = self.rev_gain * (dr * self.time_constant + r);
        let vol_p = self.kp * (r - y);
        let vol_i = self.ki * self.error_sum;
        let vol_d = self.kd * (dr - dy);
        self.error_sum = self.error_sum + (r - y) * self.period;
        vol_f + vol_p + vol_i + vol_d
    }
}

pub struct ControllerBuilder<KP, KI, KD, P, MG, MT, T> {
    kp: KP,
    ki: KI,
    kd: KD,
    period: P,
    model_gain: MG,
    model_time_constant: MT,
    _phantom: PhantomData<fn() -> T>,
}

impl<T> ControllerBuilder<(), (), (), (), (), (), T> {
    pub fn new() -> Self {
        Self {
            kp: (),
            ki: (),
            kd: (),
            period: (),
            model_gain: (),
            model_time_constant: (),
            _phantom: PhantomData,
        }
    }
}

impl<T> ControllerBuilder<f32, f32, f32, Time, f32, Time, T>
where
    T: TimeDifferentiable,
    dt!(T): TimeDifferentiable,
{
    pub fn build(self) -> Controller<T> {
        Controller {
            error_sum: Default::default(),
            rev_gain: Fraction::from_raw(1.0 / self.model_gain),
            time_constant: self.model_time_constant,
            kp: Fraction::from_raw(self.kp),
            ki: Fraction::from_raw(self.ki),
            kd: Fraction::from_raw(self.kd),
            period: self.period,
        }
    }
}

impl<KI, KD, P, MG, MT, T> ControllerBuilder<(), KI, KD, P, MG, MT, T> {
    pub fn kp(self, kp: f32) -> ControllerBuilder<f32, KI, KD, P, MG, MT, T> {
        ControllerBuilder {
            kp,
            ki: self.ki,
            kd: self.kd,
            period: self.period,
            model_gain: self.model_gain,
            model_time_constant: self.model_time_constant,
            _phantom: PhantomData,
        }
    }
}

impl<KP, KD, P, MG, MT, T> ControllerBuilder<KP, (), KD, P, MG, MT, T> {
    pub fn ki(self, ki: f32) -> ControllerBuilder<KP, f32, KD, P, MG, MT, T> {
        ControllerBuilder {
            kp: self.kp,
            ki,
            kd: self.kd,
            period: self.period,
            model_gain: self.model_gain,
            model_time_constant: self.model_time_constant,
            _phantom: PhantomData,
        }
    }
}

impl<KP, KI, P, MG, MT, T> ControllerBuilder<KP, KI, (), P, MG, MT, T> {
    pub fn kd(self, kd: f32) -> ControllerBuilder<KP, KI, f32, P, MG, MT, T> {
        ControllerBuilder {
            kp: self.kp,
            ki: self.ki,
            kd,
            period: self.period,
            model_gain: self.model_gain,
            model_time_constant: self.model_time_constant,
            _phantom: PhantomData,
        }
    }
}

impl<KP, KI, KD, MG, MT, T> ControllerBuilder<KP, KI, KD, (), MG, MT, T> {
    pub fn period(self, period: Time) -> ControllerBuilder<KP, KI, KD, Time, MG, MT, T> {
        ControllerBuilder {
            kp: self.kp,
            ki: self.ki,
            kd: self.kd,
            period,
            model_gain: self.model_gain,
            model_time_constant: self.model_time_constant,
            _phantom: PhantomData,
        }
    }
}

impl<KP, KI, KD, P, MT, T> ControllerBuilder<KP, KI, KD, P, (), MT, T> {
    pub fn model_gain(self, model_gain: f32) -> ControllerBuilder<KP, KI, KD, P, f32, MT, T> {
        ControllerBuilder {
            kp: self.kp,
            ki: self.ki,
            kd: self.kd,
            period: self.period,
            model_gain,
            model_time_constant: self.model_time_constant,
            _phantom: PhantomData,
        }
    }
}

impl<KP, KI, KD, P, MG, T> ControllerBuilder<KP, KI, KD, P, MG, (), T> {
    pub fn model_time_constant(
        self,
        model_time_constant: Time,
    ) -> ControllerBuilder<KP, KI, KD, P, MG, Time, T> {
        ControllerBuilder {
            kp: self.kp,
            ki: self.ki,
            kd: self.kd,
            period: self.period,
            model_gain: self.model_gain,
            model_time_constant,
            _phantom: PhantomData,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use quantities::Distance;

    fn build_translation_controller() -> Controller<Distance> {
        ControllerBuilder::new()
            .period(Time::from_seconds(0.001))
            .model_gain(1.0)
            .model_time_constant(Time::from_seconds(0.01))
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
