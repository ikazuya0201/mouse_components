use core::marker::PhantomData;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use spin::Mutex;
use uom::si::{
    f32::{Acceleration, Jerk, Length, Time, Velocity},
    length::meter,
};

use super::slalom_generator::{SlalomDirection, SlalomKind, SlalomParametersGenerator};
use super::slalom_generator::{SlalomGenerator, SlalomTrajectory};
use super::straight_generator::{StraightTrajectory, StraightTrajectoryGenerator};
use super::trajectory::{ShiftTrajectory, Target};
use super::Pose;
use crate::trajectory_managers::TrackingTrajectoryGenerator;
use crate::utils::builder::BuilderResult;
use crate::utils::math::{LibmMath, Math};
use crate::{get_or_err, impl_setter};

/// An enum for specifying a kind of trajectory of fast run.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RunKind {
    Straight(u16),
    StraightDiagonal(u16),
    Slalom(SlalomKind, SlalomDirection),
}

/// A trajectory generator for fast run.
pub struct RunTrajectoryGenerator<M, Generator> {
    current_velocity: Mutex<Velocity>,
    run_slalom_velocity: Velocity,
    straight_generator: StraightTrajectoryGenerator<M>,
    slalom_generator: SlalomGenerator<M, Generator>,
}

impl<M, Generator> RunTrajectoryGenerator<M, Generator>
where
    M: Math,
    Generator: SlalomParametersGenerator,
{
    fn new(
        run_slalom_velocity: Velocity,
        max_velocity: Velocity,
        max_acceleration: Acceleration,
        max_jerk: Jerk,
        generator: Generator,
        period: Time,
    ) -> Self {
        let straight_generator =
            StraightTrajectoryGenerator::<M>::new(max_velocity, max_acceleration, max_jerk, period);
        let slalom_generator =
            SlalomGenerator::new(period, generator, max_velocity, max_acceleration, max_jerk);
        Self {
            current_velocity: Mutex::new(Default::default()),
            run_slalom_velocity,
            straight_generator,
            slalom_generator,
        }
    }
}

//NOTO: this code doesn't work if the initial command is slalom
//because trajectory does not accelerate in slalom.
//Then, we assume that the initial command is straight.
//TODO: To deal with arbitrary initial commands
//TODO: Specify start and end of trajectories by `RunKind`.
//TODO: Hold current velocity as a state.
impl<M, Generator> TrackingTrajectoryGenerator<(Pose, RunKind)>
    for RunTrajectoryGenerator<M, Generator>
where
    M: Math,
    Generator: SlalomParametersGenerator,
{
    type Target = Target;
    type Trajectory = ShiftTrajectory<RunTrajectory<M>, M>;

    fn generate(&self, (pose, kind): &(Pose, RunKind)) -> Self::Trajectory {
        let mut current_velocity = self.current_velocity.lock();
        let (trajectory, terminal_velocity) = self.generate_run_trajectory_and_terminal_velocity(
            *pose,
            *kind,
            *current_velocity,
            self.run_slalom_velocity,
        );
        *current_velocity = terminal_velocity;
        trajectory
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

impl<M, Generator> RunTrajectoryGenerator<M, Generator>
where
    M: Math,
    Generator: SlalomParametersGenerator,
{
    fn generate_run_trajectory_and_terminal_velocity(
        &self,
        pose: Pose,
        kind: RunKind,
        start_velocity: Velocity,
        end_velocity: Velocity,
    ) -> (ShiftTrajectory<RunTrajectory<M>, M>, Velocity) {
        let generate_straight = |distance: Length| {
            let (trajectory, terminal_velocity) = self
                .straight_generator
                .generate_with_terminal_velocity(distance, start_velocity, end_velocity);
            (RunTrajectory::Straight(trajectory), terminal_velocity)
        };
        let (trajectory, terminal_velocity) = match kind {
            RunKind::Straight(len) => {
                let distance = len as f32 * Length::new::<meter>(0.09);
                generate_straight(distance)
            }
            RunKind::StraightDiagonal(len) => {
                let distance = len as f32 * Length::new::<meter>(0.0636396);
                generate_straight(distance)
            }
            RunKind::Slalom(kind, dir) => {
                let (trajectory, terminal_velocity) = self
                    .slalom_generator
                    .generate_slalom_with_terminal_velocity(
                        kind,
                        dir,
                        start_velocity,
                        end_velocity,
                    );
                (RunTrajectory::Slalom(trajectory), terminal_velocity)
            }
        };
        (ShiftTrajectory::new(pose, trajectory), terminal_velocity)
    }
}

pub struct RunTrajectoryGeneratorBuilder<M, Generator> {
    run_slalom_velocity: Option<Velocity>,
    max_velocity: Option<Velocity>,
    max_acceleration: Option<Acceleration>,
    max_jerk: Option<Jerk>,
    parameters_generator: Option<Generator>,
    period: Option<Time>,
    _math: PhantomData<fn() -> M>,
}

impl<Generator> Default for RunTrajectoryGeneratorBuilder<LibmMath, Generator> {
    fn default() -> Self {
        Self::new()
    }
}

impl<M, Generator> RunTrajectoryGeneratorBuilder<M, Generator> {
    pub fn new() -> Self {
        Self {
            run_slalom_velocity: None,
            max_velocity: None,
            max_acceleration: None,
            max_jerk: None,
            parameters_generator: None,
            period: None,
            _math: PhantomData,
        }
    }

    pub fn build(&mut self) -> BuilderResult<RunTrajectoryGenerator<M, Generator>>
    where
        M: Math,
        Generator: SlalomParametersGenerator,
    {
        Ok(RunTrajectoryGenerator::<M, Generator>::new(
            get_or_err!(self.run_slalom_velocity),
            get_or_err!(self.max_velocity),
            get_or_err!(self.max_acceleration),
            get_or_err!(self.max_jerk),
            get_or_err!(self.parameters_generator),
            get_or_err!(self.period),
        ))
    }

    impl_setter!(run_slalom_velocity: Velocity);
    impl_setter!(max_velocity: Velocity);
    impl_setter!(max_acceleration: Acceleration);
    impl_setter!(max_jerk: Jerk);
    impl_setter!(period: Time);
    impl_setter!(parameters_generator: Generator);
}
