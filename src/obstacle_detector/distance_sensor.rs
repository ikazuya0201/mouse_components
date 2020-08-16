use quantities::Distance;

use crate::agent::Pose;
use crate::utils::sample::Sample;

pub trait DistanceSensor {
    type Error;

    fn pose(&self) -> Pose;
    fn get_distance(&mut self) -> nb::Result<Sample<Distance>, Self::Error>;
}
