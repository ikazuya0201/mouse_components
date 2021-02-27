//! Default implementation of resource and its builder.

use crate::utils::builder::{ok_or, BuilderResult};

/// An resource type for initialization of operators.
pub struct Resource<
    'a,
    LeftEncoder,
    RightEncoder,
    Imu,
    LeftMotor,
    RightMotor,
    DistanceSensors,
    WallManager,
> {
    left_encoder: LeftEncoder,
    right_encoder: RightEncoder,
    imu: Imu,
    left_motor: LeftMotor,
    right_motor: RightMotor,
    distance_sensors: DistanceSensors,
    wall_manager: &'a WallManager,
}

impl<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensors, WallManager>
    Into<(
        &'a WallManager,
        (
            (LeftEncoder, RightEncoder, Imu),
            (LeftMotor, RightMotor),
            (&'a WallManager, DistanceSensors),
        ),
    )>
    for Resource<
        'a,
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensors,
        WallManager,
    >
{
    fn into(
        self,
    ) -> (
        &'a WallManager,
        (
            (LeftEncoder, RightEncoder, Imu),
            (LeftMotor, RightMotor),
            (&'a WallManager, DistanceSensors),
        ),
    ) {
        let Resource {
            left_encoder,
            right_encoder,
            imu,
            left_motor,
            right_motor,
            distance_sensors,
            wall_manager,
        } = self;
        (
            wall_manager,
            (
                (left_encoder, right_encoder, imu),
                (left_motor, right_motor),
                (wall_manager, distance_sensors),
            ),
        )
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
    WallManager,
> {
    left_encoder: Option<LeftEncoder>,
    right_encoder: Option<RightEncoder>,
    imu: Option<Imu>,
    left_motor: Option<LeftMotor>,
    right_motor: Option<RightMotor>,
    distance_sensors: Option<DistanceSensors>,
    wall_manager: Option<&'a WallManager>,
}

macro_rules! impl_setter {
    ($field_name: ident: $type: ty) => {
        pub fn $field_name(&mut self, $field_name: $type) -> &mut Self {
            self.$field_name = Some($field_name);
            self
        }
    };
}

impl<'a, LeftEncoder, RightEncoder, Imu, LeftMotor, RightMotor, DistanceSensors, WallManager>
    ResourceBuilder<
        'a,
        LeftEncoder,
        RightEncoder,
        Imu,
        LeftMotor,
        RightMotor,
        DistanceSensors,
        WallManager,
    >
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
        Resource<
            'a,
            LeftEncoder,
            RightEncoder,
            Imu,
            LeftMotor,
            RightMotor,
            DistanceSensors,
            WallManager,
        >,
    > {
        macro_rules! get {
            ($field_name: ident) => {{
                ok_or(self.$field_name.take(), core::stringify!($field_name))?
            }};
        }
        Ok(Resource {
            left_encoder: get!(left_encoder),
            right_encoder: get!(right_encoder),
            left_motor: get!(left_motor),
            right_motor: get!(right_motor),
            imu: get!(imu),
            distance_sensors: get!(distance_sensors),
            wall_manager: get!(wall_manager),
        })
    }

    impl_setter!(left_encoder: LeftEncoder);
    impl_setter!(right_encoder: RightEncoder);
    impl_setter!(imu: Imu);
    impl_setter!(left_motor: LeftMotor);
    impl_setter!(right_motor: RightMotor);
    impl_setter!(distance_sensors: DistanceSensors);
    impl_setter!(wall_manager: &'a WallManager);
}
