use serde::{Deserialize, Serialize};
use spin::Mutex;
use uom::si::f32::{Acceleration, Jerk, Length, Time, Velocity};

use super::slalom_generator::{
    DefaultSlalomParametersGenerator, SlalomDirection, SlalomKind, SlalomParametersGenerator,
};
use super::slalom_generator::{SlalomGenerator, SlalomTrajectory};
use super::straight_generator::{StraightTrajectory, StraightTrajectoryGenerator};
use super::trajectory::{ShiftTrajectory, Target};
use super::Pose;
use crate::trajectory_managers::TrackingTrajectoryGenerator;
use crate::utils::builder::BuilderResult;
use crate::{get_or_err, impl_setter};
use crate::{impl_deconstruct_with_default, Construct};

/// An enum for describing a state of run.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum RunState {
    Acceleration,
    Constant,
    // First phase of deceleration.
    Deceleration1,
    // Second phase of deceleration.
    Deceleration2,
}

/// An enum for specifying a kind of trajectory of fast run.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum RunKind {
    Straight(u8),
    StraightDiagonal(u8),
    Slalom(SlalomKind, SlalomDirection),
}

/// A trajectory generator for fast run.
pub struct RunTrajectoryGenerator<Generator> {
    current_velocity: Mutex<Velocity>,
    run_slalom_velocity: Velocity,
    straight_generator: StraightTrajectoryGenerator,
    slalom_generator: SlalomGenerator<Generator>,
    square_width: Length,
    square_diagonal_half: Length,
}

impl<Generator> RunTrajectoryGenerator<Generator>
where
    Generator: SlalomParametersGenerator,
{
    const HALF_DIAGONAL: f32 = 0.70710677;

    fn new(
        run_slalom_velocity: Velocity,
        max_velocity: Velocity,
        max_acceleration: Acceleration,
        max_jerk: Jerk,
        generator: Generator,
        period: Time,
        square_width: Length,
    ) -> Self {
        let straight_generator =
            StraightTrajectoryGenerator::new(max_velocity, max_acceleration, max_jerk, period);
        let slalom_generator =
            SlalomGenerator::new(period, generator, max_velocity, max_acceleration, max_jerk);
        Self {
            current_velocity: Mutex::new(Default::default()),
            run_slalom_velocity,
            straight_generator,
            slalom_generator,
            square_width,
            square_diagonal_half: square_width * Self::HALF_DIAGONAL,
        }
    }
}

/// Config for [RunTrajectoryGenerator].
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct RunTrajectoryGeneratorConfig {
    pub run_slalom_velocity: Velocity,
    pub max_velocity: Velocity,
    pub max_acceleration: Acceleration,
    pub max_jerk: Jerk,
    pub period: Time,
    pub square_width: Length,
    pub front_offset: Length,
}

impl<Config, State, Resource> Construct<Config, State, Resource>
    for RunTrajectoryGenerator<DefaultSlalomParametersGenerator>
where
    Config: AsRef<RunTrajectoryGeneratorConfig>,
{
    fn construct<'a>(config: &'a Config, _state: &'a State, _resource: &'a mut Resource) -> Self {
        let config = config.as_ref();
        Self::new(
            config.run_slalom_velocity,
            config.max_velocity,
            config.max_acceleration,
            config.max_jerk,
            DefaultSlalomParametersGenerator::new(config.square_width, config.front_offset),
            config.period,
            config.square_width,
        )
    }
}

impl_deconstruct_with_default!(RunTrajectoryGenerator<DefaultSlalomParametersGenerator>);

/// A parameter type for [RunTrajectoryGenerator].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RunTrajectoryParameters {
    pub pose: Pose,
    pub kind: RunKind,
    pub state: RunState,
}

//NOTO: this code doesn't work if the initial command is slalom
//because trajectory does not accelerate in slalom.
//Then, we assume that the initial command is straight.
//TODO: To deal with arbitrary initial commands
//TODO: Specify start and end of trajectories by `RunKind`.
//TODO: Hold current velocity as a state.
impl<Generator> TrackingTrajectoryGenerator<RunTrajectoryParameters>
    for RunTrajectoryGenerator<Generator>
where
    Generator: SlalomParametersGenerator,
{
    type Target = Target;
    type Trajectory = ShiftTrajectory<RunTrajectory>;

    fn generate(&self, parameters: &RunTrajectoryParameters) -> Self::Trajectory {
        let mut current_velocity = self.current_velocity.lock();
        let (start, end) = match parameters.state {
            RunState::Acceleration => (*current_velocity, self.run_slalom_velocity),
            RunState::Constant => (self.run_slalom_velocity, self.run_slalom_velocity),
            RunState::Deceleration1 => (*current_velocity, *current_velocity / 2.0),
            RunState::Deceleration2 => (*current_velocity, Default::default()),
        };
        let generate_straight = |distance: Length| {
            let (trajectory, terminal_velocity) = self
                .straight_generator
                .generate_with_terminal_velocity(distance, start, end);
            (RunTrajectory::Straight(trajectory), terminal_velocity)
        };
        let (trajectory, terminal_velocity) = match parameters.kind {
            RunKind::Straight(len) => {
                let distance = len as f32 * self.square_width;
                generate_straight(distance)
            }
            RunKind::StraightDiagonal(len) => {
                let distance = len as f32 * self.square_diagonal_half;
                generate_straight(distance)
            }
            RunKind::Slalom(kind, dir) => {
                let (trajectory, terminal_velocity) = self
                    .slalom_generator
                    .generate_slalom_with_terminal_velocity(kind, dir, start, end);
                (RunTrajectory::Slalom(trajectory), terminal_velocity)
            }
        };
        *current_velocity = terminal_velocity;
        ShiftTrajectory::new(parameters.pose, trajectory)
    }
}

pub enum RunTrajectory {
    Straight(StraightTrajectory),
    Slalom(SlalomTrajectory),
}

impl Iterator for RunTrajectory {
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            RunTrajectory::Straight(inner) => inner.next(),
            RunTrajectory::Slalom(inner) => inner.next(),
        }
    }
}

pub struct RunTrajectoryGeneratorBuilder<Generator> {
    run_slalom_velocity: Option<Velocity>,
    max_velocity: Option<Velocity>,
    max_acceleration: Option<Acceleration>,
    max_jerk: Option<Jerk>,
    parameters_generator: Option<Generator>,
    period: Option<Time>,
    square_width: Option<Length>,
}

impl<Generator> Default for RunTrajectoryGeneratorBuilder<Generator> {
    fn default() -> Self {
        Self::new()
    }
}

impl<Generator> RunTrajectoryGeneratorBuilder<Generator> {
    pub fn new() -> Self {
        Self {
            run_slalom_velocity: None,
            max_velocity: None,
            max_acceleration: None,
            max_jerk: None,
            parameters_generator: None,
            period: None,
            square_width: None,
        }
    }

    pub fn build(&mut self) -> BuilderResult<RunTrajectoryGenerator<Generator>>
    where
        Generator: SlalomParametersGenerator,
    {
        Ok(RunTrajectoryGenerator::<Generator>::new(
            get_or_err!(self.run_slalom_velocity),
            get_or_err!(self.max_velocity),
            get_or_err!(self.max_acceleration),
            get_or_err!(self.max_jerk),
            get_or_err!(self.parameters_generator),
            get_or_err!(self.period),
            get_or_err!(self.square_width),
        ))
    }

    impl_setter!(run_slalom_velocity: Velocity);
    impl_setter!(max_velocity: Velocity);
    impl_setter!(max_acceleration: Acceleration);
    impl_setter!(max_jerk: Jerk);
    impl_setter!(period: Time);
    impl_setter!(parameters_generator: Generator);
    impl_setter!(square_width: Length);
}
