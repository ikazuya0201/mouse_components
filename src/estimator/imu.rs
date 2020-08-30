use quantities::{Acceleration, AngularSpeed};

///The y axis should directed to the front of the robot.
///The ground should be the x-y plane.
pub trait IMU {
    type Error;

    fn get_translational_acceleration(&mut self) -> nb::Result<Acceleration, Self::Error>;
    fn get_angular_speed(&mut self) -> nb::Result<AngularSpeed, Self::Error>;
}
