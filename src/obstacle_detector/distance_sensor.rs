use crate::agent::Pose;
use crate::quantities::f32::Length;
use crate::utils::sample::Sample;

pub trait DistanceSensor {
    type Error;

    fn pose(&self) -> Pose;
    fn get_distance(&mut self) -> nb::Result<Sample<Length>, Self::Error>;
}
