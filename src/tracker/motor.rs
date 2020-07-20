use quantities::Voltage;

pub trait Motor {
    fn apply(&mut self, voltage: Voltage);
}
