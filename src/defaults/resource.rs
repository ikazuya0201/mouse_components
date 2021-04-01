//! Default implementation of resource and its builder.

use heapless::Vec;

use crate::get_or_err;
use crate::obstacle_detector::SensorSizeUpperBound;
use crate::utils::builder::BuilderResult;

/// An resource type for initialization of operators.
pub struct Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor> {
    pub left_encoder: LeftEncoder,
    pub right_encoder: RightEncoder,
    pub imu: Imu,
    pub left_motor: LeftMotor,
    pub right_motor: RightMotor,
    pub distance_sensors: Vec<DistanceSensor, SensorSizeUpperBound>,
}

/// A builder for [Resource](Resource).
pub struct ResourceBuilder<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor> {
    left_encoder: Option<LeftEncoder>,
    right_encoder: Option<RightEncoder>,
    imu: Option<Imu>,
    left_motor: Option<LeftMotor>,
    right_motor: Option<RightMotor>,
    distance_sensors: Option<Vec<DistanceSensor, SensorSizeUpperBound>>,
}

macro_rules! impl_setter {
    ($field_name: ident: $type: ty) => {
        pub fn $field_name(&mut self, $field_name: $type) -> &mut Self {
            self.$field_name = Some($field_name);
            self
        }
    };
}

impl<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor>
    ResourceBuilder<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor>
{
    pub fn new() -> Self {
        Self {
            left_encoder: None,
            right_encoder: None,
            imu: None,
            left_motor: None,
            right_motor: None,
            distance_sensors: None,
        }
    }

    pub fn build(
        &mut self,
    ) -> BuilderResult<
        Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor>,
    > {
        Ok(Resource {
            left_encoder: get_or_err!(self.left_encoder),
            right_encoder: get_or_err!(self.right_encoder),
            left_motor: get_or_err!(self.left_motor),
            right_motor: get_or_err!(self.right_motor),
            imu: get_or_err!(self.imu),
            distance_sensors: get_or_err!(self.distance_sensors),
        })
    }

    impl_setter!(left_encoder: LeftEncoder);
    impl_setter!(right_encoder: RightEncoder);
    impl_setter!(imu: Imu);
    impl_setter!(left_motor: LeftMotor);
    impl_setter!(right_motor: RightMotor);
    impl_setter!(distance_sensors: Vec<DistanceSensor, SensorSizeUpperBound>);
}
