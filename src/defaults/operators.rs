pub mod initialize;
mod return_setup_operator;
mod run_operator;
mod search_operator;

pub use return_setup_operator::ReturnSetupOperator;
pub use run_operator::{ReturnOperator, RunOperator};
pub use search_operator::SearchOperator;

#[cfg(test)]
mod mocks {
    use core::convert::Infallible;

    use uom::si::f32::{Acceleration, AngularVelocity, ElectricPotential, Length};

    use crate::sensors::{DistanceSensor, Encoder, Motor, IMU};
    use crate::types::data::Pose;
    use crate::utils::sample::Sample;

    pub struct EncoderMock;

    impl Encoder for EncoderMock {
        type Error = Infallible;

        fn get_relative_distance(&mut self) -> nb::Result<Length, Self::Error> {
            unimplemented!()
        }
    }

    pub struct MotorMock;

    impl Motor for MotorMock {
        fn apply(&mut self, _electric_potential: ElectricPotential) {
            unimplemented!()
        }
    }

    pub struct ImuMock;

    impl IMU for ImuMock {
        type Error = Infallible;

        fn get_angular_velocity(&mut self) -> nb::Result<AngularVelocity, Self::Error> {
            unimplemented!()
        }

        fn get_translational_acceleration(&mut self) -> nb::Result<Acceleration, Self::Error> {
            unimplemented!()
        }
    }

    pub struct DistanceSensorMock;

    impl DistanceSensor for DistanceSensorMock {
        type Error = Infallible;

        fn pose(&self) -> Pose {
            unimplemented!()
        }

        fn get_distance(&mut self) -> nb::Result<Sample<Length>, Self::Error> {
            unimplemented!()
        }
    }
}
