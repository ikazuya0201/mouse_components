//! Default implementation of resource and its builder.

use core::ops::Mul;

use generic_array::ArrayLength;
use spin::Mutex;
use typenum::consts::U2;

use crate::utils::builder::{ok_or, BuilderResult};
use crate::utils::probability::Probability;
use crate::wall_manager::WallManager;

/// An resource type for initialization of operators.
pub struct Resource<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensors,
    Size,
> where
    Size: Mul<Size>,
    Size::Output: Mul<U2>,
    <Size::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
{
    left_encoder: Option<LeftEncoder>,
    right_encoder: Option<RightEncoder>,
    imu: Option<Imu>,
    left_motor: Option<LeftMotor>,
    right_motor: Option<RightMotor>,
    distance_sensors: Option<DistanceSensors>,
    wall_manager: &'a WallManager<Size>,
}

impl<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensors, Size>
    Resource<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensors, Size>
where
    Size: Mul<Size>,
    Size::Output: Mul<U2>,
    <Size::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
{
    pub fn take_left_encoder(&mut self) -> Option<LeftEncoder> {
        self.left_encoder.take()
    }

    pub fn take_right_encoder(&mut self) -> Option<RightEncoder> {
        self.right_encoder.take()
    }

    pub fn take_left_motor(&mut self) -> Option<LeftMotor> {
        self.left_motor.take()
    }

    pub fn take_right_motor(&mut self) -> Option<RightMotor> {
        self.right_motor.take()
    }

    pub fn take_imu(&mut self) -> Option<Imu> {
        self.imu.take()
    }

    pub fn take_distance_sensors(&mut self) -> Option<DistanceSensors> {
        self.distance_sensors.take()
    }

    pub fn wall_manager(&self) -> &WallManager<Size> {
        self.wall_manager
    }
}

/// A builder for [Resource](Resource).
pub struct ResourceBuilder<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensors,
    Size,
> where
    Size: Mul<Size>,
    Size::Output: Mul<U2>,
    <Size::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
{
    left_encoder: Option<LeftEncoder>,
    right_encoder: Option<RightEncoder>,
    imu: Option<Imu>,
    left_motor: Option<LeftMotor>,
    right_motor: Option<RightMotor>,
    distance_sensors: Option<DistanceSensors>,
    wall_manager: Option<&'a WallManager<Size>>,
}

macro_rules! impl_setter {
    ($field_name: ident: $type: ty) => {
        pub fn $field_name(&mut self, $field_name: $type) -> &mut Self {
            self.$field_name = Some($field_name);
            self
        }
    };
}

impl<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensors, Size>
    ResourceBuilder<
        'a,
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensors,
        Size,
    >
where
    Size: Mul<Size>,
    Size::Output: Mul<U2>,
    <Size::Output as Mul<U2>>::Output: ArrayLength<Mutex<Probability>>,
{
    pub fn new() -> Self {
        Self {
            left_encoder: None,
            right_encoder: None,
            imu: None,
            left_motor: None,
            right_motor: None,
            distance_sensors: None,
            wall_manager: None,
        }
    }

    pub fn build(
        &mut self,
    ) -> BuilderResult<
        Resource<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensors, Size>,
    > {
        Ok(Resource {
            left_encoder: self.left_encoder.take(),
            right_encoder: self.right_encoder.take(),
            left_motor: self.left_motor.take(),
            right_motor: self.right_motor.take(),
            imu: self.imu.take(),
            distance_sensors: self.distance_sensors.take(),
            wall_manager: ok_or(self.wall_manager.take(), "wall_manager")?,
        })
    }

    impl_setter!(left_encoder: LeftEncoder);
    impl_setter!(right_encoder: RightEncoder);
    impl_setter!(imu: Imu);
    impl_setter!(left_motor: LeftMotor);
    impl_setter!(right_motor: RightMotor);
    impl_setter!(distance_sensors: DistanceSensors);
    impl_setter!(wall_manager: &'a WallManager<Size>);
}
