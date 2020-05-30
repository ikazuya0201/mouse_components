use quantities::Distance;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DistanceSensorError;

pub trait DistanceSensor {
    fn get_distance(&mut self) -> nb::Result<Distance, DistanceSensorError>;
}
