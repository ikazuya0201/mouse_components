use embedded_hal::PwmPin;
use uom::si::{electric_potential::volt, f32::ElectricPotential, ratio::ratio};

pub struct Motor<P1, P2>
where
    P1: PwmPin,
    P2: PwmPin,
{
    pwm_pin1: P1,
    pwm_pin2: P2,
}

impl<P1, P2> Motor<P1, P2>
where
    P1: PwmPin,
    P2: PwmPin,
{
    pub fn new(pwm_pin1: P1, pwm_pin2: P2) -> Self {
        Self { pwm_pin1, pwm_pin2 }
    }
}

impl<P1, P2> Motor<P1, P2>
where
    P1: PwmPin<Duty = u16>,
    P2: PwmPin<Duty = u16>,
{
    pub fn apply(&mut self, voltage: ElectricPotential, battery_voltage: ElectricPotential) {
        if voltage.get::<volt>() > 0.0 {
            let mut duty_ratio = (voltage / battery_voltage).get::<ratio>();
            if duty_ratio > 1.0 {
                duty_ratio = 1.0;
            }
            self.pwm_pin1
                .set_duty(((1.0 - duty_ratio) * self.pwm_pin1.get_max_duty() as f32) as u16);
            self.pwm_pin2.set_duty(self.pwm_pin2.get_max_duty());
        } else {
            let mut duty_ratio = -(voltage / battery_voltage).get::<ratio>();
            if duty_ratio > 1.0 {
                duty_ratio = 1.0;
            }
            self.pwm_pin1.set_duty(self.pwm_pin1.get_max_duty());
            self.pwm_pin2
                .set_duty(((1.0 - duty_ratio) * self.pwm_pin2.get_max_duty() as f32) as u16);
        }
    }
}
