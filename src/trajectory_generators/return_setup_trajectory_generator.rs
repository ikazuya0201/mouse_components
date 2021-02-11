use uom::si::angle::degree;
use uom::si::f32::{Angle, AngularAcceleration, AngularJerk, AngularVelocity, Time};

use super::spin_generator::{SpinGenerator, SpinTrajectory};
use super::{ok_or, RequiredFieldEmptyError, Target};
use crate::nodes::RotationKind;
use crate::trajectory_managers::InitialTrajectoryGenerator;
use crate::utils::math::Math;

/// An implementation of
/// [InitialTrajectoryGenerator](crate::trajectory_managers::InitialTrajectoryGenerator).
pub struct ReturnSetupTrajectoryGenerator<M> {
    spin_generator: SpinGenerator<M>,
}

impl<M: Math> InitialTrajectoryGenerator<RotationKind> for ReturnSetupTrajectoryGenerator<M> {
    type MaxLength = typenum::consts::U1;
    type Target = Target;
    type Trajectory = SpinTrajectory;
    type Trajectories = Option<Self::Trajectory>;

    fn generate<Commands: IntoIterator<Item = RotationKind>>(
        &self,
        commands: Commands,
    ) -> Self::Trajectories {
        commands.into_iter().next().map(|command| {
            use RotationKind::*;

            let theta = match command {
                Front => Default::default(),
                Right => Angle::new::<degree>(-90.0),
                Left => Angle::new::<degree>(90.0),
                Back => Angle::new::<degree>(180.0),
            };
            self.spin_generator.generate(Default::default(), theta)
        })
    }
}

/// A builder for [ReturnSetupTrajectoryGenerator](ReturnSetupTrajectoryGenerator).
pub struct ReturnSetupTrajectoryGeneratorBuilder {
    max_angular_velocity: Option<AngularVelocity>,
    max_angular_acceleration: Option<AngularAcceleration>,
    max_angular_jerk: Option<AngularJerk>,
    period: Option<Time>,
}

impl ReturnSetupTrajectoryGeneratorBuilder {
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

    pub fn build<M: Math>(
        self,
    ) -> Result<ReturnSetupTrajectoryGenerator<M>, RequiredFieldEmptyError> {
        Ok(ReturnSetupTrajectoryGenerator {
            spin_generator: SpinGenerator::<M>::new(
                ok_or(self.max_angular_velocity, "max_angular_velocity")?,
                ok_or(self.max_angular_acceleration, "max_angular_acceleration")?,
                ok_or(self.max_angular_jerk, "max_angular_jerk")?,
                ok_or(self.period, "period")?,
            ),
        })
    }
}
