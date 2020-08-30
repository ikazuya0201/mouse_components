use crate::quantities::f32::Voltage;

pub trait Motor {
    fn apply(&mut self, voltage: Voltage);
}
