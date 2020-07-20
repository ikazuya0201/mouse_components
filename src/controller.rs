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
