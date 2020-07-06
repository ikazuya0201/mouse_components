use crate::utils::vector::Vector3;

use quantities::{Acceleration, AngularSpeed};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

pub type AngularSpeeds = Vector3<AngularSpeed>;
pub type Accelerations = Vector3<Acceleration>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IMUError;

pub trait IMU {
    fn get_angular_speed(&mut self, axis: Axis) -> nb::Result<AngularSpeed, IMUError>;
    fn get_acceleration(&mut self, axis: Axis) -> nb::Result<Acceleration, IMUError>;

    fn get_angular_speeds(&mut self) -> nb::Result<AngularSpeeds, IMUError> {
        Ok(AngularSpeeds {
            x: self.get_angular_speed(Axis::X)?,
            y: self.get_angular_speed(Axis::Y)?,
            z: self.get_angular_speed(Axis::Z)?,
        })
    }

    fn get_accelerations(&mut self) -> nb::Result<Accelerations, IMUError> {
        Ok(Accelerations {
            x: self.get_acceleration(Axis::X)?,
            y: self.get_acceleration(Axis::Y)?,
            z: self.get_acceleration(Axis::Z)?,
        })
    }
}
