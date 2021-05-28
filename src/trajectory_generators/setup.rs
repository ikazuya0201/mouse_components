use serde::{Deserialize, Serialize};
use uom::si::angle::degree;
use uom::si::f32::{Angle, AngularAcceleration, AngularJerk, AngularVelocity, Time};

use super::spin::{SpinGenerator, SpinTrajectory};
use super::{ShiftTrajectory, SingleTrajectory, Target};
use crate::nodes::RotationKind;
use crate::trajectory_managers::TrackingTrajectoryGenerator;
use crate::types::data::Pose;
use crate::utils::builder::{ok_or, RequiredFieldEmptyError};
use crate::{impl_deconstruct_with_default, Construct};

/// An implementation of
/// [TrackingTrajectoryGenerator](crate::trajectory_managers::TrackingTrajectoryGenerator).
pub struct ReturnSetupTrajectoryGenerator {
    spin_generator: SpinGenerator,
}

impl ReturnSetupTrajectoryGenerator {
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
        Self { spin_generator }
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

impl<Config, State, Resource> Construct<Config, State, Resource> for ReturnSetupTrajectoryGenerator
where
    Config: AsRef<ReturnSetupTrajectoryGeneratorConfig>,
{
    fn construct<'a>(config: &'a Config, _state: &'a State, _resource: &'a mut Resource) -> Self {
        let config = config.as_ref();
        Self::new(
            config.max_angular_velocity,
            config.max_angular_acceleration,
            config.max_angular_jerk,
            config.period,
        )
    }
}

impl_deconstruct_with_default!(ReturnSetupTrajectoryGenerator);

pub enum SetupTrajectory {
    Empty(SingleTrajectory),
    Rotation(SpinTrajectory),
}

impl Iterator for SetupTrajectory {
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Self::Empty(inner) => inner.next(),
            Self::Rotation(inner) => inner.next(),
        }
    }
}

//TODO: Write test.
impl TrackingTrajectoryGenerator<(Pose, RotationKind)> for ReturnSetupTrajectoryGenerator {
    type Target = Target;
    type Trajectory = ShiftTrajectory<SetupTrajectory>;

    fn generate(&self, &(pose, kind): &(Pose, RotationKind)) -> Self::Trajectory {
        use RotationKind::*;

        let theta = match kind {
            Front => {
                return ShiftTrajectory::new(
                    pose,
                    SetupTrajectory::Empty(SingleTrajectory::new(Default::default())),
                )
            }
            Right => Angle::new::<degree>(-90.0),
            Left => Angle::new::<degree>(90.0),
            Back => Angle::new::<degree>(180.0),
        };
        ShiftTrajectory::new(
            pose,
            SetupTrajectory::Rotation(self.spin_generator.generate(theta)),
        )
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

    pub fn build(self) -> Result<ReturnSetupTrajectoryGenerator, RequiredFieldEmptyError> {
        Ok(ReturnSetupTrajectoryGenerator::new(
            ok_or(self.max_angular_velocity, "max_angular_velocity")?,
            ok_or(self.max_angular_acceleration, "max_angular_acceleration")?,
            ok_or(self.max_angular_jerk, "max_angular_jerk")?,
            ok_or(self.period, "period")?,
        ))
    }
}
