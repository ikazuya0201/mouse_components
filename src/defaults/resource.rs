//! Default implementation of resource and its builder.

use alloc::rc::Rc;
use core::convert::TryInto;

use heapless::Vec;

use crate::get_or_err;
use crate::obstacle_detector::SENSOR_SIZE_UPPER_BOUND;
use crate::types::resources::*;
use crate::utils::builder::{ok_or, BuilderResult, RequiredFieldEmptyError};
use crate::wall_manager::WallManager;
use crate::Merge;

/// An resource type for initialization of operators.
pub struct Resource<
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    const N: usize,
> {
    pub left_encoder: Option<LeftEncoder>,
    pub right_encoder: Option<RightEncoder>,
    pub imu: Option<Imu>,
    pub left_motor: Option<LeftMotor>,
    pub right_motor: Option<RightMotor>,
    pub distance_sensors: Option<Vec<DistanceSensor, SENSOR_SIZE_UPPER_BOUND>>,
    pub wall_manager: Option<Rc<WallManager<N>>>,
}

impl<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, const N: usize> Default
    for Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>
{
    fn default() -> Self {
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
}

macro_rules! impl_try_into_and_from {
    ($name: ident $(<$($type: ty),*>)? {
        $($field_name: ident,)*
    }) => {
        impl<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, const N: usize>
            TryInto<(
                Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>,
                $name$(<$($type),*>)?,
            )> for Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>
        {
            type Error = RequiredFieldEmptyError;

            fn try_into(
                mut self,
            ) -> Result<
                (
                    Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>,
                    $name$(<$($type),*>)?,
                ),
                Self::Error,
            > {
                $(
                    let $field_name = get_or_err!(self.$field_name);
                )*
                Ok((
                    self,
                    $name {
                        $($field_name,)*
                    },
                ))
            }
        }

        impl<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, const N: usize>
            From<$name $(<$($type),*>)?> for Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>
        {
            fn from(value: $name $(<$($type),*>)?) -> Self {
                let mut resource = Resource::default();
                let $name {
                    $($field_name,)*
                } = value;
                $(resource.$field_name = Some($field_name);)*
                resource
            }
        }
    }
}

impl_try_into_and_from! {
    EstimatorResource<LeftEncoder, RightEncoder, Imu> {
        left_encoder,
        right_encoder,
        imu,
    }
}

impl_try_into_and_from! {
    TrackerResource<LeftMotor, RightMotor> {
        left_motor,
        right_motor,
    }
}

impl_try_into_and_from! {
    ObstacleDetectorResource<DistanceSensor> {
        distance_sensors,
    }
}

impl<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, const N: usize>
    TryInto<(
        Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>,
        Rc<WallManager<N>>,
    )> for Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>
{
    type Error = RequiredFieldEmptyError;

    fn try_into(
        self,
    ) -> Result<
        (
            Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>,
            Rc<WallManager<N>>,
        ),
        Self::Error,
    > {
        let wall_manager = Rc::clone(ok_or(self.wall_manager.as_ref(), "wall_manager")?);
        Ok((self, wall_manager))
    }
}

impl<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, const N: usize>
    From<Rc<WallManager<N>>>
    for Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>
{
    fn from(value: Rc<WallManager<N>>) -> Self {
        let mut resource = Self::default();
        resource.wall_manager = Some(value);
        resource
    }
}

impl<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, const N: usize> Merge
    for Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>
{
    fn merge(mut self, mut rhs: Self) -> Self {
        // prioritize left hand side value.
        macro_rules! or {
            ($name: ident) => {
                self.$name = self.$name.or(rhs.$name.take());
            };
        }
        or!(left_encoder);
        or!(right_encoder);
        or!(imu);
        or!(left_motor);
        or!(right_motor);
        or!(distance_sensors);
        or!(wall_manager);
        self
    }
}

/// A builder for [Resource](Resource).
pub struct ResourceBuilder<
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    const N: usize,
> {
    left_encoder: Option<LeftEncoder>,
    right_encoder: Option<RightEncoder>,
    imu: Option<Imu>,
    left_motor: Option<LeftMotor>,
    right_motor: Option<RightMotor>,
    distance_sensors: Option<Vec<DistanceSensor, SENSOR_SIZE_UPPER_BOUND>>,
    wall_manager: Option<Rc<WallManager<N>>>,
}

macro_rules! impl_setter {
    ($field_name: ident: $type: ty) => {
        pub fn $field_name(&mut self, $field_name: $type) -> &mut Self {
            self.$field_name = Some($field_name);
            self
        }
    };
}

impl<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, const N: usize>
    ResourceBuilder<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>
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
        Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>,
    > {
        Ok(Resource {
            left_encoder: self.left_encoder.take(),
            right_encoder: self.right_encoder.take(),
            left_motor: self.left_motor.take(),
            right_motor: self.right_motor.take(),
            imu: self.imu.take(),
            distance_sensors: self.distance_sensors.take(),
            wall_manager: self.wall_manager.take(),
        })
    }

    impl_setter!(left_encoder: LeftEncoder);
    impl_setter!(right_encoder: RightEncoder);
    impl_setter!(imu: Imu);
    impl_setter!(left_motor: LeftMotor);
    impl_setter!(right_motor: RightMotor);
    impl_setter!(distance_sensors: Vec<DistanceSensor, SENSOR_SIZE_UPPER_BOUND>);
    impl_setter!(wall_manager: Rc<WallManager<N>>);
}
