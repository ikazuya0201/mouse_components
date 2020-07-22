use crate::utils::vector::Vector3;

use quantities::{Acceleration, AngularSpeed};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IMUError;

///The y axis should directed to the front of the robot.
///The ground should be the x-y plane.
pub trait IMU {
    fn get_angular_speed_x(&mut self) -> nb::Result<AngularSpeed, IMUError>;
    fn get_angular_speed_y(&mut self) -> nb::Result<AngularSpeed, IMUError>;
    fn get_angular_speed_z(&mut self) -> nb::Result<AngularSpeed, IMUError>;

    fn get_acceleration_x(&mut self) -> nb::Result<Acceleration, IMUError>;
    fn get_acceleration_y(&mut self) -> nb::Result<Acceleration, IMUError>;
    fn get_acceleration_z(&mut self) -> nb::Result<Acceleration, IMUError>;

    fn get_angular_speeds(&mut self) -> nb::Result<Vector3<AngularSpeed>, IMUError> {
        Ok(Vector3 {
            x: self.get_angular_speed_x()?,
            y: self.get_angular_speed_y()?,
            z: self.get_angular_speed_z()?,
        })
    }

    fn get_accelerations(&mut self) -> nb::Result<Vector3<Acceleration>, IMUError> {
        Ok(Vector3 {
            x: self.get_acceleration_x()?,
            y: self.get_acceleration_y()?,
            z: self.get_acceleration_z()?,
        })
    }
}
