use quantities::Distance;

use crate::agent::Pose;

pub trait DistanceSensor {
    type Error;

    fn pose(&self) -> Pose;
    fn get_distance(&mut self) -> nb::Result<Distance, Self::Error>;
}
