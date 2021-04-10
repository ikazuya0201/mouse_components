use core::marker::PhantomData;

use serde::{Deserialize, Serialize};
use uom::si::angle::degree;
use uom::si::f32::{Angle, AngularAcceleration, AngularJerk, AngularVelocity, Time};

use super::spin_generator::{SpinGenerator, SpinTrajectory};
use super::{ShiftTrajectory, Target};
use crate::nodes::RotationKind;
use crate::trajectory_managers::TrackingTrajectoryGenerator;
use crate::types::data::Pose;
use crate::utils::builder::{ok_or, RequiredFieldEmptyError};
use crate::{impl_deconstruct_with_default, Construct};

/// An implementation of
/// [TrackingTrajectoryGenerator](crate::trajectory_managers::TrackingTrajectoryGenerator).
pub struct ReturnSetupTrajectoryGenerator<Math> {
    spin_generator: SpinGenerator,
    _math: PhantomData<fn() -> Math>,
}

impl<Math> ReturnSetupTrajectoryGenerator<Math> {
    pub fn new(
        max_angular_velocity: AngularVelocity,
        max_angular_acceleration: AngularAcceleration,
        max_angular_jerk: AngularJerk,
        period: Time,
    ) -> Self {
        let spin_generator = SpinGenerator::new(
            max_angular_velocity,
            max_angular_acceleration,
            max_angular_jerk,
            period,
        );
        Self {
            spin_generator,
            _math: PhantomData,
        }
    }
}

/// Config for [ReturnSetupTrajectoryGenerator].
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct ReturnSetupTrajectoryGeneratorConfig {
    pub max_angular_velocity: AngularVelocity,
    pub max_angular_acceleration: AngularAcceleration,
    pub max_angular_jerk: AngularJerk,
    pub period: Time,
}

impl<Math, Config, State, Resource> Construct<Config, State, Resource>
    for ReturnSetupTrajectoryGenerator<Math>
where
    Config: AsRef<ReturnSetupTrajectoryGeneratorConfig>,
{
    fn construct(config: &Config, _state: &State, resource: Resource) -> (Self, Resource) {
        let config = config.as_ref();
        (
            Self::new(
                config.max_angular_velocity,
                config.max_angular_acceleration,
                config.max_angular_jerk,
                config.period,
            ),
            resource,
        )
    }
}

impl_deconstruct_with_default!(ReturnSetupTrajectoryGenerator<Math>);

//TODO: Write test.
impl<Math> TrackingTrajectoryGenerator<(Pose, RotationKind)>
    for ReturnSetupTrajectoryGenerator<Math>
where
    Math: crate::utils::math::Math,
{
    type Target = Target;
    type Trajectory = ShiftTrajectory<SpinTrajectory, Math>;

    fn generate(&self, &(pose, kind): &(Pose, RotationKind)) -> Self::Trajectory {
        use RotationKind::*;

        let theta = match kind {
            Front => Default::default(),
            Right => Angle::new::<degree>(-90.0),
            Left => Angle::new::<degree>(90.0),
            Back => Angle::new::<degree>(180.0),
        };
        ShiftTrajectory::new(pose, self.spin_generator.generate(theta))
    }
}

/// A builder for [ReturnSetupTrajectoryGenerator](ReturnSetupTrajectoryGenerator).
pub struct ReturnSetupTrajectoryGeneratorBuilder {
    max_angular_velocity: Option<AngularVelocity>,
    max_angular_acceleration: Option<AngularAcceleration>,
    max_angular_jerk: Option<AngularJerk>,
    period: Option<Time>,
}

impl Default for ReturnSetupTrajectoryGeneratorBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl ReturnSetupTrajectoryGeneratorBuilder {
    pub fn new() -> Self {
        Self {
            max_angular_velocity: None,
            max_angular_acceleration: None,
            max_angular_jerk: None,
            period: None,
        }
    }

    pub fn max_angular_velocity(mut self, max_angular_velocity: AngularVelocity) -> Self {
        self.max_angular_velocity = Some(max_angular_velocity);
        self
    }

    pub fn max_angular_acceleration(
        mut self,
        max_angular_acceleration: AngularAcceleration,
    ) -> Self {
        self.max_angular_acceleration = Some(max_angular_acceleration);
        self
    }

    pub fn max_angular_jerk(mut self, max_angular_jerk: AngularJerk) -> Self {
        self.max_angular_jerk = Some(max_angular_jerk);
        self
    }

    pub fn period(mut self, period: Time) -> Self {
        self.period = Some(period);
        self
    }

    pub fn build<Math>(
        self,
    ) -> Result<ReturnSetupTrajectoryGenerator<Math>, RequiredFieldEmptyError> {
        Ok(ReturnSetupTrajectoryGenerator::new(
            ok_or(self.max_angular_velocity, "max_angular_velocity")?,
            ok_or(self.max_angular_acceleration, "max_angular_acceleration")?,
            ok_or(self.max_angular_jerk, "max_angular_jerk")?,
            ok_or(self.period, "period")?,
        ))
    }
}
