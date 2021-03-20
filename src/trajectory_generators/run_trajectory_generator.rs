use core::marker::PhantomData;

use heapless::{ArrayLength, Vec};

use super::slalom_generator::{SlalomDirection, SlalomKind, SlalomParameters};
use super::slalom_generator::{SlalomGenerator, SlalomTrajectory};
use super::straight_generator::{StraightTrajectory, StraightTrajectoryGenerator};
use super::trajectory::{ShiftTrajectory, Target};
use super::Pose;
use crate::trajectory_managers::InitialTrajectoryGenerator;
use crate::utils::builder::{ok_or, RequiredFieldEmptyError};
use crate::utils::math::{LibmMath, Math};
use uom::si::{
    f32::{
        Acceleration, AngularAcceleration, AngularJerk, AngularVelocity, Jerk, Length, Time,
        Velocity,
    },
    length::meter,
};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RunKind {
    Straight(u16),
    StraightDiagonal(u16),
    Slalom(SlalomKind, SlalomDirection),
}

pub struct RunTrajectoryGenerator<M, MaxLength> {
    run_slalom_velocity: Velocity,
    straight_generator: StraightTrajectoryGenerator<M>,
    slalom_generator: SlalomGenerator<M>,
    _max_length: PhantomData<fn() -> MaxLength>,
}

impl<M: Math, MaxLength> RunTrajectoryGenerator<M, MaxLength> {
    fn new(
        run_slalom_velocity: Velocity,
        max_velocity: Velocity,
        max_acceleration: Acceleration,
        max_jerk: Jerk,
        angular_velocity_ref: AngularVelocity,
        angular_acceleration_ref: AngularAcceleration,
        angular_jerk_ref: AngularJerk,
        slalom_parameters_map: fn(SlalomKind, SlalomDirection) -> SlalomParameters,
        period: Time,
    ) -> Self {
        let straight_generator =
            StraightTrajectoryGenerator::<M>::new(max_velocity, max_acceleration, max_jerk, period);
        let slalom_generator = SlalomGenerator::new(
            angular_velocity_ref,
            angular_acceleration_ref,
            angular_jerk_ref,
            period,
            slalom_parameters_map,
        );
        Self {
            run_slalom_velocity,
            straight_generator,
            slalom_generator,
            _max_length: PhantomData,
        }
    }
}

//NOTO: this code doesn't work if the initial command is slalom
//because trajectory does not accelerate in slalom.
//Then, we assume that the initial command is straight.
//TODO: To deal with arbitrary initial commands
impl<M, MaxLength> InitialTrajectoryGenerator<(Pose, RunKind)>
    for RunTrajectoryGenerator<M, MaxLength>
where
    M: Math,
    MaxLength: ArrayLength<ShiftTrajectory<RunTrajectory<M>, M>>,
{
    type Target = Target;
    type Trajectory = ShiftTrajectory<RunTrajectory<M>, M>;
    type Trajectories = Vec<Self::Trajectory, MaxLength>;

    fn generate<Commands: IntoIterator<Item = (Pose, RunKind)>>(
        &self,
        commands: Commands,
    ) -> Self::Trajectories {
        let mut current_velocity = Default::default();
        let mut trajectories = Vec::new();
        for (pose, kind) in commands {
            let (trajectory, next_velocity) =
                self.generate_run_trajectory_and_terminal_velocity(&pose, &kind, &current_velocity);
            current_velocity = next_velocity;
            trajectories.push(trajectory).unwrap_or_else(|_| {
                unreachable!("The length of trajectories should be properly upper-bounded.")
            });
        }
        trajectories
    }
}

pub enum RunTrajectory<M> {
    Straight(StraightTrajectory),
    Slalom(SlalomTrajectory<M>),
}

impl<M> Iterator for RunTrajectory<M>
where
    M: Math,
{
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            RunTrajectory::Straight(inner) => inner.next(),
            RunTrajectory::Slalom(inner) => inner.next(),
        }
    }
}

impl<M, MaxLength> RunTrajectoryGenerator<M, MaxLength>
where
    M: Math,
{
    fn generate_run_trajectory_and_terminal_velocity(
        &self,
        pose: &Pose,
        kind: &RunKind,
        velocity: &Velocity,
    ) -> (ShiftTrajectory<RunTrajectory<M>, M>, Velocity) {
        let generate_straight = |distance: Length| {
            let terminal_velocity = self.straight_generator.reachable_velocity(
                distance,
                *velocity,
                self.run_slalom_velocity,
            );
            (
                RunTrajectory::Straight(self.straight_generator.generate(
                    distance,
                    *velocity,
                    terminal_velocity,
                )),
                terminal_velocity,
            )
        };
        let (trajectory, terminal_velocity) = match kind {
            RunKind::Straight(len) => {
                let distance = *len as f32 * Length::new::<meter>(0.09);
                generate_straight(distance)
            }
            RunKind::StraightDiagonal(len) => {
                let distance = *len as f32 * Length::new::<meter>(0.0636396);
                generate_straight(distance)
            }
            RunKind::Slalom(kind, dir) => (
                RunTrajectory::Slalom(
                    self.slalom_generator
                        .generate_slalom(*kind, *dir, *velocity),
                ),
                *velocity,
            ),
        };
        (ShiftTrajectory::new(*pose, trajectory), terminal_velocity)
    }
}

pub struct RunTrajectoryGeneratorBuilder<M = LibmMath> {
    run_slalom_velocity: Option<Velocity>,
    max_velocity: Option<Velocity>,
    max_acceleration: Option<Acceleration>,
    max_jerk: Option<Jerk>,
    angular_velocity_ref: Option<AngularVelocity>,
    angular_acceleration_ref: Option<AngularAcceleration>,
    angular_jerk_ref: Option<AngularJerk>,
    slalom_parameters_map: Option<fn(SlalomKind, SlalomDirection) -> SlalomParameters>,
    period: Option<Time>,
    _math: PhantomData<fn() -> M>,
}

impl Default for RunTrajectoryGeneratorBuilder<LibmMath> {
    fn default() -> Self {
        Self::new()
    }
}

impl<M> RunTrajectoryGeneratorBuilder<M> {
    pub fn new() -> Self {
        Self {
            run_slalom_velocity: None,
            max_velocity: None,
            max_acceleration: None,
            max_jerk: None,
            angular_velocity_ref: None,
            angular_acceleration_ref: None,
            angular_jerk_ref: None,
            slalom_parameters_map: None,
            period: None,
            _math: PhantomData,
        }
    }

    pub fn build<MaxLength>(
        self,
    ) -> Result<RunTrajectoryGenerator<M, MaxLength>, RequiredFieldEmptyError>
    where
        M: Math,
    {
        Ok(RunTrajectoryGenerator::<M, MaxLength>::new(
            ok_or(self.run_slalom_velocity, "run_slalom_velocity")?,
            ok_or(self.max_velocity, "max_velocity")?,
            ok_or(self.max_acceleration, "max_acceleration")?,
            ok_or(self.max_jerk, "max_jerk")?,
            ok_or(self.angular_velocity_ref, "angular_velocity_ref")?,
            ok_or(self.angular_acceleration_ref, "angular_acceleration_ref")?,
            ok_or(self.angular_jerk_ref, "angular_jerk_ref")?,
            ok_or(self.slalom_parameters_map, "slalom_parameters_map")?,
            ok_or(self.period, "period")?,
        ))
    }

    pub fn run_slalom_velocity(mut self, run_slalom_velocity: Velocity) -> Self {
        self.run_slalom_velocity = Some(run_slalom_velocity);
        self
    }

    pub fn max_velocity(mut self, max_velocity: Velocity) -> Self {
        self.max_velocity = Some(max_velocity);
        self
    }

    pub fn max_acceleration(mut self, max_acceleration: Acceleration) -> Self {
        self.max_acceleration = Some(max_acceleration);
        self
    }

    pub fn max_jerk(mut self, max_jerk: Jerk) -> Self {
        self.max_jerk = Some(max_jerk);
        self
    }

    pub fn period(mut self, period: Time) -> Self {
        self.period = Some(period);
        self
    }

    pub fn angular_velocity_ref(mut self, angular_velocity_ref: AngularVelocity) -> Self {
        self.angular_velocity_ref = Some(angular_velocity_ref);
        self
    }

    pub fn angular_acceleration_ref(
        mut self,
        angular_acceleration_ref: AngularAcceleration,
    ) -> Self {
        self.angular_acceleration_ref = Some(angular_acceleration_ref);
        self
    }

    pub fn angular_jerk_ref(mut self, angular_jerk_ref: AngularJerk) -> Self {
        self.angular_jerk_ref = Some(angular_jerk_ref);
        self
    }

    pub fn slalom_parameters_map(
        mut self,
        slalom_parameters_map: fn(SlalomKind, SlalomDirection) -> SlalomParameters,
    ) -> Self {
        self.slalom_parameters_map = Some(slalom_parameters_map);
        self
    }
}
