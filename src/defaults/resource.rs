//! Default implementation of resource and its builder.

use alloc::rc::Rc;

use heapless::Vec;

use crate::obstacle_detector::SENSOR_SIZE_UPPER_BOUND;
use crate::types::resources::*;
use crate::utils::builder::BuilderResult;
use crate::wall_manager::WallManager;
use crate::{get_or_err, Merge};

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
    pub left_encoder: LeftEncoder,
    pub right_encoder: RightEncoder,
    pub imu: Imu,
    pub left_motor: LeftMotor,
    pub right_motor: RightMotor,
    pub distance_sensors: Vec<DistanceSensor, SENSOR_SIZE_UPPER_BOUND>,
    pub wall_manager: Rc<WallManager<N>>,
}

macro_rules! impl_from {
    ($name: ident $(<$($type: ty),*>)? {
        $($field_name: ident,)*
    }) => {
        impl<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, const N: usize>
            From<$name $(<$($type),*>)?> for ResourceBuilder<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>
        {
            fn from(value: $name $(<$($type),*>)?) -> Self {
                let mut resource = ResourceBuilder::default();
                let $name {
                    $($field_name,)*
                } = value;
                $(resource.$field_name = Some($field_name);)*
                resource
            }
        }
    }
}

impl_from! {
    EstimatorResource<LeftEncoder, RightEncoder, Imu> {
        left_encoder,
        right_encoder,
        imu,
    }
}

impl_from! {
    TrackerResource<LeftMotor, RightMotor> {
        left_motor,
        right_motor,
    }
}

impl_from! {
    ObstacleDetectorResource<DistanceSensor> {
        distance_sensors,
    }
}

impl<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, const N: usize>
    From<Rc<WallManager<N>>>
    for ResourceBuilder<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>
{
    fn from(value: Rc<WallManager<N>>) -> Self {
        let mut resource = Self::default();
        resource.wall_manager = Some(value);
        resource
    }
}

pub struct ResourceContainer<
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensor,
    const N: usize,
> {
    estimator: Option<EstimatorResource<LeftEncoder, RightEncoder, Imu>>,
    tracker: Option<TrackerResource<LeftMotor, RightMotor>>,
    obstacle_detector: Option<ObstacleDetectorResource<DistanceSensor>>,
    wall_manager: Rc<WallManager<N>>,
}

macro_rules! impl_as_mut {
    ($field_name: ident: $name: ident $(<$($type: ty),*>)?) => {
        impl<
                LeftEncoder,
                RightEncoder,
                Imu,
                LeftMotor,
                RightMotor,
                DistanceSensor,
                const N: usize,
            > AsMut<Option<$name $(<$($type),*>)?>>
            for ResourceContainer<
                LeftEncoder,
                RightEncoder,
                Imu,
                LeftMotor,
                RightMotor,
                DistanceSensor,
                N,
            >
        {
            fn as_mut(&mut self) -> &mut Option<$name $(<$($type),*>)?> {
                &mut self.$field_name
            }
        }
    };
}

impl_as_mut!(estimator: EstimatorResource<LeftEncoder, RightEncoder, Imu>);
impl_as_mut!(tracker: TrackerResource<LeftMotor, RightMotor>);
impl_as_mut!(obstacle_detector: ObstacleDetectorResource<DistanceSensor>);

impl<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, const N: usize>
    AsRef<Rc<WallManager<N>>>
    for ResourceContainer<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>
{
    fn as_ref(&self) -> &Rc<WallManager<N>> {
        &self.wall_manager
    }
}

impl<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, const N: usize>
    Into<
        ResourceContainer<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>,
    > for Resource<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>
{
    fn into(
        self,
    ) -> ResourceContainer<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>
    {
        let Resource {
            left_encoder,
            right_encoder,
            imu,
            left_motor,
            right_motor,
            distance_sensors,
            wall_manager,
        } = self;
        ResourceContainer {
            estimator: Some(EstimatorResource {
                left_encoder,
                right_encoder,
                imu,
            }),
            tracker: Some(TrackerResource {
                left_motor,
                right_motor,
            }),
            obstacle_detector: Some(ObstacleDetectorResource { distance_sensors }),
            wall_manager,
        }
    }
}

impl<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, const N: usize> Merge
    for ResourceBuilder<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>
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

impl<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, const N: usize> Default
    for ResourceBuilder<LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensor, N>
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
            left_encoder: get_or_err!(self.left_encoder),
            right_encoder: get_or_err!(self.right_encoder),
            left_motor: get_or_err!(self.left_motor),
            right_motor: get_or_err!(self.right_motor),
            imu: get_or_err!(self.imu),
            distance_sensors: get_or_err!(self.distance_sensors),
            wall_manager: get_or_err!(self.wall_manager),
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
