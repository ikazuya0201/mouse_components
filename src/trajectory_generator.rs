mod slalom_generator;
mod spin_generator;
mod straight_generator;
mod trajectory;

use core::iter::Chain;
use core::marker::PhantomData;

use heapless::{ArrayLength, Vec};

use crate::agent::{RunTrajectoryGenerator, SearchTrajectoryGenerator};
use crate::data_types::Pose;
use crate::traits::Math;
pub use slalom_generator::{
    slalom_parameters_map, slalom_parameters_map2, SlalomDirection, SlalomKind, SlalomParameters,
};
use slalom_generator::{SlalomGenerator, SlalomTrajectory};
use spin_generator::{SpinGenerator, SpinTrajectory};
use straight_generator::{StraightTrajectory, StraightTrajectoryGenerator};
use trajectory::StopTrajectory;
pub use trajectory::{AngleTarget, LengthTarget, MoveTarget, ShiftTrajectory, Target};
use uom::si::{
    angle::degree,
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, Jerk, Length, Time,
        Velocity,
    },
    length::meter,
};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SearchKind {
    Init,
    Final,
    Front,
    Right,
    Left,
    Back,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RunKind {
    Straight(u16),
    StraightDiagonal(u16),
    Slalom(SlalomKind, SlalomDirection),
}

pub enum SearchTrajectory<M> {
    Straight(StraightTrajectory),
    Slalom(SlalomTrajectory<M>),
    Back(BackTrajectory<M>),
    SlowDown(SlowDownTrajectory),
}

impl<M: Math> Iterator for SearchTrajectory<M> {
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        use SearchTrajectory::*;

        match self {
            Straight(inner) => inner.next(),
            Slalom(inner) => inner.next(),
            Back(inner) => inner.next(),
            SlowDown(inner) => inner.next(),
        }
    }

    fn advance_by(&mut self, n: usize) -> Result<(), usize> {
        use SearchTrajectory::*;

        match self {
            Straight(inner) => inner.advance_by(n),
            Slalom(inner) => inner.advance_by(n),
            Back(inner) => inner.advance_by(n),
            SlowDown(inner) => inner.advance_by(n),
        }
    }
}

pub type BackTrajectory<M> = Chain<
    Chain<
        Chain<
            Chain<Chain<Chain<StraightTrajectory, StopTrajectory>, SpinTrajectory>, StopTrajectory>,
            SpinTrajectory,
        >,
        StopTrajectory,
    >,
    ShiftTrajectory<StraightTrajectory, M>,
>;

pub type SlowDownTrajectory = Chain<StraightTrajectory, StopTrajectory>;

pub struct TrajectoryGenerator<M, MaxLength> {
    straight_generator: StraightTrajectoryGenerator<M>,

    #[allow(unused)]
    slalom_generator: SlalomGenerator<M>,
    #[allow(unused)]
    spin_generator: SpinGenerator<M>,

    run_slalom_velocity: Velocity,

    initial_trajectory: StraightTrajectory,
    slow_down_trajectory: SlowDownTrajectory,
    front_trajectory: StraightTrajectory,
    right_trajectory: SlalomTrajectory<M>,
    left_trajectory: SlalomTrajectory<M>,
    back_trajectory: BackTrajectory<M>,

    _max_length: PhantomData<fn() -> MaxLength>,
}

impl<M: Math, MaxLength> TrajectoryGenerator<M, MaxLength> {
    fn new(
        angular_velocity_ref: AngularVelocity,
        angular_acceleration_ref: AngularAcceleration,
        angular_jerk_ref: AngularJerk,
        slalom_parameters_map: fn(SlalomKind, SlalomDirection) -> SlalomParameters,
        max_velocity: Velocity,
        max_acceleration: Acceleration,
        max_jerk: Jerk,
        period: Time,
        search_velocity: Velocity,
        run_slalom_velocity: Velocity,
        front_offset: Length,
        square_width: Length,
        spin_angular_velocity: AngularVelocity,
        spin_angular_acceleration: AngularAcceleration,
        spin_angular_jerk: AngularJerk,
    ) -> Self {
        let straight_generator =
            StraightTrajectoryGenerator::new(max_velocity, max_acceleration, max_jerk, period);
        let slalom_generator = SlalomGenerator::new(
            angular_velocity_ref,
            angular_acceleration_ref,
            angular_jerk_ref,
            period,
            slalom_parameters_map,
        );
        let spin_generator = SpinGenerator::new(
            spin_angular_velocity,
            spin_angular_acceleration,
            spin_angular_jerk,
            period,
        );

        let square_width_half = square_width / 2.0;

        let initial_trajectory = straight_generator.generate(
            square_width_half + front_offset,
            Default::default(),
            search_velocity,
        );
        let slow_down_trajectory = straight_generator
            .generate(
                square_width_half - front_offset,
                search_velocity,
                Default::default(),
            )
            .chain(StopTrajectory::new(
                Pose {
                    x: square_width_half - front_offset,
                    ..Default::default()
                },
                period,
                Time::new::<second>(1.5),
            ));
        let right_trajectory = slalom_generator.generate_slalom(
            SlalomKind::Search90,
            SlalomDirection::Right,
            search_velocity,
        );
        let left_trajectory = slalom_generator.generate_slalom(
            SlalomKind::Search90,
            SlalomDirection::Left,
            search_velocity,
        );
        //TODO: use configurable value
        let front_trajectory =
            straight_generator.generate(square_width, search_velocity, search_velocity);
        use uom::si::time::{millisecond, second};
        let back_trajectory = {
            let distance = square_width_half - front_offset; //TODO: use configurable value
            straight_generator
                .generate(distance, search_velocity, Default::default())
                .chain(StopTrajectory::new(
                    Pose {
                        x: distance,
                        ..Default::default()
                    },
                    period,
                    Time::new::<millisecond>(50.0),
                ))
                .chain(spin_generator.generate(Default::default(), Angle::new::<degree>(90.0)))
                .chain(StopTrajectory::new(
                    Pose {
                        x: distance,
                        y: Default::default(),
                        theta: Angle::new::<degree>(90.0),
                    },
                    period,
                    Time::new::<millisecond>(50.0),
                ))
                .chain(
                    spin_generator.generate(Angle::new::<degree>(90.0), Angle::new::<degree>(90.0)),
                )
                .chain(StopTrajectory::new(
                    Pose {
                        x: distance,
                        y: Default::default(),
                        theta: Angle::new::<degree>(180.0),
                    },
                    period,
                    Time::new::<millisecond>(50.0),
                ))
                .chain(ShiftTrajectory::new(
                    Pose {
                        x: distance,
                        y: Default::default(),
                        theta: Angle::new::<degree>(180.0),
                    },
                    straight_generator.generate(
                        distance + 2.0 * front_offset,
                        Default::default(),
                        search_velocity,
                    ),
                ))
        };

        Self {
            straight_generator,
            slalom_generator,
            spin_generator,
            initial_trajectory,
            slow_down_trajectory,
            front_trajectory,
            right_trajectory,
            left_trajectory,
            back_trajectory,
            run_slalom_velocity,
            _max_length: PhantomData,
        }
    }
}

impl<M, MaxLength> SearchTrajectoryGenerator<(Pose, SearchKind)>
    for TrajectoryGenerator<M, MaxLength>
where
    M: Math,
{
    type Target = Target;
    type Trajectory = ShiftTrajectory<SearchTrajectory<M>, M>;

    fn generate_search(&self, (pose, kind): &(Pose, SearchKind)) -> Self::Trajectory {
        self.generate_search_trajectory(pose, kind)
    }

    fn generate_emergency(&self, target: &Self::Target) -> Self::Trajectory {
        //This method should never be called when spin.
        let target = target.moving().expect("Should be Target::Moving.");
        let pose = Pose {
            x: target.x.x,
            y: target.y.x,
            theta: target.theta.x,
        };
        self.generate_search_trajectory(&pose, &SearchKind::Final)
    }
}

//NOTO: this code doesn't work if the initial command is slalom
//because trajectory does not accelerate in slalom.
//Then, we assume that the initial command is straight.
//TODO: To deal with arbitrary initial commands
impl<M, MaxLength> RunTrajectoryGenerator<(Pose, RunKind)> for TrajectoryGenerator<M, MaxLength>
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

impl<M, MaxLength> TrajectoryGenerator<M, MaxLength>
where
    M: Math,
{
    fn generate_search_trajectory(
        &self,
        pose: &Pose,
        kind: &SearchKind,
    ) -> ShiftTrajectory<SearchTrajectory<M>, M> {
        use SearchKind::*;

        ShiftTrajectory::<_, M>::new(
            *pose,
            match kind {
                Init => SearchTrajectory::Straight(self.initial_trajectory.clone()),
                Final => SearchTrajectory::SlowDown(self.slow_down_trajectory.clone()),
                Front => SearchTrajectory::Straight(self.front_trajectory.clone()),
                Right => SearchTrajectory::Slalom(self.right_trajectory.clone()),
                Left => SearchTrajectory::Slalom(self.left_trajectory.clone()),
                Back => SearchTrajectory::Back(self.back_trajectory.clone()),
            },
        )
    }

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

    pub fn generate_straight(
        &self,
        distance: Length,
        v_start: Velocity,
        v_end: Velocity,
    ) -> impl Iterator<Item = Target> {
        self.straight_generator.generate(distance, v_start, v_end)
    }

    pub fn generate_spin(
        &self,
        theta_start: Angle,
        theta_distance: Angle,
    ) -> impl Iterator<Item = Target> {
        self.spin_generator.generate(theta_start, theta_distance)
    }
}

pub struct TrajectoryGeneratorBuilder<
    AngularVelocityRef,
    AngularAccelerationRef,
    AngularJerkRef,
    SlalomParametersMap,
    MaxVelocity,
    MaxAcceleration,
    MaxJerk,
    Period,
    SearchVelocity,
    RunSlalomVelocity,
    Offset,
    SquareWidth,
    SpinAngularVelocity,
    SpinAngularAcceleration,
    SpinAngularJerk,
> {
    angular_velocity_ref: AngularVelocityRef,
    angular_acceleration_ref: AngularAccelerationRef,
    angular_jerk_ref: AngularJerkRef,
    slalom_parameters_map: SlalomParametersMap,
    max_velocity: MaxVelocity,
    max_acceleration: MaxAcceleration,
    max_jerk: MaxJerk,
    period: Period,
    search_velocity: SearchVelocity,
    run_slalom_velocity: RunSlalomVelocity,
    front_offset: Offset,
    square_width: SquareWidth,
    spin_angular_velocity: SpinAngularVelocity,
    spin_angular_acceleration: SpinAngularAcceleration,
    spin_angular_jerk: SpinAngularJerk,
}

impl<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
{
    const DEFAULT_SQUARE_WIDTH: Length = Length {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.09,
    };

    const DEFAULT_FRONT_OFFSET: Length = Length {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.0,
    };
}

impl TrajectoryGeneratorBuilder<(), (), (), (), (), (), (), (), (), (), (), (), (), (), ()> {
    pub fn new() -> Self {
        Self {
            angular_velocity_ref: (),
            angular_acceleration_ref: (),
            angular_jerk_ref: (),
            slalom_parameters_map: (),
            max_velocity: (),
            max_acceleration: (),
            max_jerk: (),
            period: (),
            search_velocity: (),
            run_slalom_velocity: (),
            front_offset: (),
            square_width: (),
            spin_angular_velocity: (),
            spin_angular_acceleration: (),
            spin_angular_jerk: (),
        }
    }
}

impl
    TrajectoryGeneratorBuilder<
        AngularVelocity,
        AngularAcceleration,
        AngularJerk,
        fn(SlalomKind, SlalomDirection) -> SlalomParameters,
        Velocity,
        Acceleration,
        Jerk,
        Time,
        Velocity,
        Velocity,
        (),
        (),
        AngularVelocity,
        AngularAcceleration,
        AngularJerk,
    >
{
    pub fn build<M: Math, MaxLength>(self) -> TrajectoryGenerator<M, MaxLength> {
        TrajectoryGenerator::new(
            self.angular_velocity_ref,
            self.angular_acceleration_ref,
            self.angular_jerk_ref,
            self.slalom_parameters_map,
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.period,
            self.search_velocity,
            self.run_slalom_velocity,
            Self::DEFAULT_FRONT_OFFSET,
            Self::DEFAULT_SQUARE_WIDTH,
            self.spin_angular_velocity,
            self.spin_angular_acceleration,
            self.spin_angular_jerk,
        )
    }
}

impl
    TrajectoryGeneratorBuilder<
        AngularVelocity,
        AngularAcceleration,
        AngularJerk,
        fn(SlalomKind, SlalomDirection) -> SlalomParameters,
        Velocity,
        Acceleration,
        Jerk,
        Time,
        Velocity,
        Velocity,
        (),
        Length,
        AngularVelocity,
        AngularAcceleration,
        AngularJerk,
    >
{
    pub fn build<M: Math, MaxLength>(self) -> TrajectoryGenerator<M, MaxLength> {
        TrajectoryGenerator::new(
            self.angular_velocity_ref,
            self.angular_acceleration_ref,
            self.angular_jerk_ref,
            self.slalom_parameters_map,
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.period,
            self.search_velocity,
            self.run_slalom_velocity,
            Self::DEFAULT_FRONT_OFFSET,
            self.square_width,
            self.spin_angular_velocity,
            self.spin_angular_acceleration,
            self.spin_angular_jerk,
        )
    }
}

impl
    TrajectoryGeneratorBuilder<
        AngularVelocity,
        AngularAcceleration,
        AngularJerk,
        fn(SlalomKind, SlalomDirection) -> SlalomParameters,
        Velocity,
        Acceleration,
        Jerk,
        Time,
        Velocity,
        Velocity,
        Length,
        (),
        AngularVelocity,
        AngularAcceleration,
        AngularJerk,
    >
{
    pub fn build<M: Math, MaxLength>(self) -> TrajectoryGenerator<M, MaxLength> {
        TrajectoryGenerator::new(
            self.angular_velocity_ref,
            self.angular_acceleration_ref,
            self.angular_jerk_ref,
            self.slalom_parameters_map,
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.period,
            self.search_velocity,
            self.run_slalom_velocity,
            self.front_offset,
            Self::DEFAULT_SQUARE_WIDTH,
            self.spin_angular_velocity,
            self.spin_angular_acceleration,
            self.spin_angular_jerk,
        )
    }
}

impl
    TrajectoryGeneratorBuilder<
        AngularVelocity,
        AngularAcceleration,
        AngularJerk,
        fn(SlalomKind, SlalomDirection) -> SlalomParameters,
        Velocity,
        Acceleration,
        Jerk,
        Time,
        Velocity,
        Velocity,
        Length,
        Length,
        AngularVelocity,
        AngularAcceleration,
        AngularJerk,
    >
{
    pub fn build<M: Math, MaxLength>(self) -> TrajectoryGenerator<M, MaxLength> {
        TrajectoryGenerator::new(
            self.angular_velocity_ref,
            self.angular_acceleration_ref,
            self.angular_jerk_ref,
            self.slalom_parameters_map,
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.period,
            self.search_velocity,
            self.run_slalom_velocity,
            self.front_offset,
            self.square_width,
            self.spin_angular_velocity,
            self.spin_angular_acceleration,
            self.spin_angular_jerk,
        )
    }
}

impl<
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        (),
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
{
    pub fn angular_velocity_ref(
        self,
        angular_velocity_ref: AngularVelocity,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocity,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_parameters_map: self.slalom_parameters_map,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
            run_slalom_velocity: self.run_slalom_velocity,
            front_offset: self.front_offset,
            square_width: self.square_width,
            spin_angular_velocity: self.spin_angular_velocity,
            spin_angular_acceleration: self.spin_angular_acceleration,
            spin_angular_jerk: self.spin_angular_jerk,
        }
    }
}

impl<
        AngularVelocityRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        (),
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
{
    pub fn angular_acceleration_ref(
        self,
        angular_acceleration_ref: AngularAcceleration,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAcceleration,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_parameters_map: self.slalom_parameters_map,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
            run_slalom_velocity: self.run_slalom_velocity,
            front_offset: self.front_offset,
            square_width: self.square_width,
            spin_angular_velocity: self.spin_angular_velocity,
            spin_angular_acceleration: self.spin_angular_acceleration,
            spin_angular_jerk: self.spin_angular_jerk,
        }
    }
}

impl<
        AngularVelocityRef,
        AngularAccelerationRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        (),
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
{
    pub fn angular_jerk_ref(
        self,
        angular_jerk_ref: AngularJerk,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerk,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref,
            slalom_parameters_map: self.slalom_parameters_map,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
            run_slalom_velocity: self.run_slalom_velocity,
            front_offset: self.front_offset,
            square_width: self.square_width,
            spin_angular_velocity: self.spin_angular_velocity,
            spin_angular_acceleration: self.spin_angular_acceleration,
            spin_angular_jerk: self.spin_angular_jerk,
        }
    }
}

impl<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        (),
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
{
    pub fn slalom_parameters_map(
        self,
        slalom_parameters_map: fn(SlalomKind, SlalomDirection) -> SlalomParameters,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        fn(SlalomKind, SlalomDirection) -> SlalomParameters,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_parameters_map,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
            run_slalom_velocity: self.run_slalom_velocity,
            front_offset: self.front_offset,
            square_width: self.square_width,
            spin_angular_velocity: self.spin_angular_velocity,
            spin_angular_acceleration: self.spin_angular_acceleration,
            spin_angular_jerk: self.spin_angular_jerk,
        }
    }
}

impl<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        (),
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
{
    pub fn max_velocity(
        self,
        max_velocity: Velocity,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        Velocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_parameters_map: self.slalom_parameters_map,
            max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
            run_slalom_velocity: self.run_slalom_velocity,
            front_offset: self.front_offset,
            square_width: self.square_width,
            spin_angular_velocity: self.spin_angular_velocity,
            spin_angular_acceleration: self.spin_angular_acceleration,
            spin_angular_jerk: self.spin_angular_jerk,
        }
    }
}

impl<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        (),
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
{
    pub fn max_acceleration(
        self,
        max_acceleration: Acceleration,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        Acceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_parameters_map: self.slalom_parameters_map,
            max_velocity: self.max_velocity,
            max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
            run_slalom_velocity: self.run_slalom_velocity,
            front_offset: self.front_offset,
            square_width: self.square_width,
            spin_angular_velocity: self.spin_angular_velocity,
            spin_angular_acceleration: self.spin_angular_acceleration,
            spin_angular_jerk: self.spin_angular_jerk,
        }
    }
}

impl<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        (),
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
{
    pub fn max_jerk(
        self,
        max_jerk: Jerk,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        Jerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_parameters_map: self.slalom_parameters_map,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
            run_slalom_velocity: self.run_slalom_velocity,
            front_offset: self.front_offset,
            square_width: self.square_width,
            spin_angular_velocity: self.spin_angular_velocity,
            spin_angular_acceleration: self.spin_angular_acceleration,
            spin_angular_jerk: self.spin_angular_jerk,
        }
    }
}

impl<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        (),
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
{
    pub fn period(
        self,
        period: Time,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Time,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_parameters_map: self.slalom_parameters_map,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period,
            search_velocity: self.search_velocity,
            run_slalom_velocity: self.run_slalom_velocity,
            front_offset: self.front_offset,
            square_width: self.square_width,
            spin_angular_velocity: self.spin_angular_velocity,
            spin_angular_acceleration: self.spin_angular_acceleration,
            spin_angular_jerk: self.spin_angular_jerk,
        }
    }
}

impl<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        (),
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
{
    pub fn search_velocity(
        self,
        search_velocity: Velocity,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        Velocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_parameters_map: self.slalom_parameters_map,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity,
            run_slalom_velocity: self.run_slalom_velocity,
            front_offset: self.front_offset,
            square_width: self.square_width,
            spin_angular_velocity: self.spin_angular_velocity,
            spin_angular_acceleration: self.spin_angular_acceleration,
            spin_angular_jerk: self.spin_angular_jerk,
        }
    }
}

impl<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        (),
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
{
    pub fn run_slalom_velocity(
        self,
        run_slalom_velocity: Velocity,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        Velocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_parameters_map: self.slalom_parameters_map,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
            run_slalom_velocity,
            front_offset: self.front_offset,
            square_width: self.square_width,
            spin_angular_velocity: self.spin_angular_velocity,
            spin_angular_acceleration: self.spin_angular_acceleration,
            spin_angular_jerk: self.spin_angular_jerk,
        }
    }
}

impl<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        (),
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
{
    pub fn front_offset(
        self,
        front_offset: Length,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Length,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_parameters_map: self.slalom_parameters_map,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
            run_slalom_velocity: self.run_slalom_velocity,
            front_offset,
            square_width: self.square_width,
            spin_angular_velocity: self.spin_angular_velocity,
            spin_angular_acceleration: self.spin_angular_acceleration,
            spin_angular_jerk: self.spin_angular_jerk,
        }
    }
}

impl<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        (),
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
{
    pub fn square_width(
        self,
        square_width: Length,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        Length,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_parameters_map: self.slalom_parameters_map,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
            run_slalom_velocity: self.run_slalom_velocity,
            front_offset: self.front_offset,
            square_width,
            spin_angular_velocity: self.spin_angular_velocity,
            spin_angular_acceleration: self.spin_angular_acceleration,
            spin_angular_jerk: self.spin_angular_jerk,
        }
    }
}

impl<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        (),
        SpinAngularAcceleration,
        SpinAngularJerk,
    >
{
    pub fn spin_angular_velocity(
        self,
        spin_angular_velocity: AngularVelocity,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        AngularVelocity,
        SpinAngularAcceleration,
        SpinAngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_parameters_map: self.slalom_parameters_map,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
            run_slalom_velocity: self.run_slalom_velocity,
            front_offset: self.front_offset,
            square_width: self.square_width,
            spin_angular_velocity,
            spin_angular_acceleration: self.spin_angular_acceleration,
            spin_angular_jerk: self.spin_angular_jerk,
        }
    }
}

impl<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularJerk,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        (),
        SpinAngularJerk,
    >
{
    pub fn spin_angular_acceleration(
        self,
        spin_angular_acceleration: AngularAcceleration,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        AngularAcceleration,
        SpinAngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_parameters_map: self.slalom_parameters_map,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
            run_slalom_velocity: self.run_slalom_velocity,
            front_offset: self.front_offset,
            square_width: self.square_width,
            spin_angular_velocity: self.spin_angular_velocity,
            spin_angular_acceleration,
            spin_angular_jerk: self.spin_angular_jerk,
        }
    }
}

impl<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
    >
    TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        (),
    >
{
    pub fn spin_angular_jerk(
        self,
        spin_angular_jerk: AngularJerk,
    ) -> TrajectoryGeneratorBuilder<
        AngularVelocityRef,
        AngularAccelerationRef,
        AngularJerkRef,
        SlalomParametersMap,
        MaxVelocity,
        MaxAcceleration,
        MaxJerk,
        Period,
        SearchVelocity,
        RunSlalomVelocity,
        Offset,
        SquareWidth,
        SpinAngularVelocity,
        SpinAngularAcceleration,
        AngularJerk,
    > {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_parameters_map: self.slalom_parameters_map,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
            run_slalom_velocity: self.run_slalom_velocity,
            front_offset: self.front_offset,
            square_width: self.square_width,
            spin_angular_velocity: self.spin_angular_velocity,
            spin_angular_acceleration: self.spin_angular_acceleration,
            spin_angular_jerk,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::math::MathFake;
    use approx::assert_relative_eq;
    use uom::si::{
        acceleration::meter_per_second_squared,
        angle::radian,
        angular_acceleration::{degree_per_second_squared, radian_per_second_squared},
        angular_jerk::{degree_per_second_cubed, radian_per_second_cubed},
        angular_velocity::{degree_per_second, radian_per_second},
        jerk::meter_per_second_cubed,
        time::second,
        velocity::meter_per_second,
    };

    fn build_generator() -> TrajectoryGenerator<MathFake, ()> {
        TrajectoryGeneratorBuilder::new()
            .max_velocity(Velocity::new::<meter_per_second>(1.0))
            .max_acceleration(Acceleration::new::<meter_per_second_squared>(10.0))
            .max_jerk(Jerk::new::<meter_per_second_cubed>(100.0))
            .angular_velocity_ref(AngularVelocity::new::<degree_per_second>(180.0))
            .angular_acceleration_ref(AngularAcceleration::new::<degree_per_second_squared>(
                1800.0,
            ))
            .angular_jerk_ref(AngularJerk::new::<degree_per_second_cubed>(18000.0))
            .slalom_parameters_map(slalom_parameters_map)
            .period(Time::new::<second>(0.001))
            .search_velocity(Velocity::new::<meter_per_second>(0.6))
            .run_slalom_velocity(Velocity::new::<meter_per_second>(1.0))
            .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(90.0))
            .spin_angular_acceleration(AngularAcceleration::new::<degree_per_second_squared>(90.0))
            .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(180.0))
            .build()
    }

    #[test]
    fn test_build() {
        build_generator();
    }

    macro_rules! impl_search_trajectory_test {
        ($name: ident: $value: expr) => {
            #[test]
            fn $name() {
                use core::f32::consts::PI;

                const EPSILON: f32 = 1e-3;

                let period = Time::new::<second>(0.001);
                let search_velocity = Velocity::new::<meter_per_second>(0.2);

                let generator = TrajectoryGeneratorBuilder::new()
                    .period(period)
                    .max_velocity(Velocity::new::<meter_per_second>(2.0))
                    .max_acceleration(Acceleration::new::<meter_per_second_squared>(0.7))
                    .max_jerk(Jerk::new::<meter_per_second_cubed>(1.0))
                    .search_velocity(search_velocity)
                    .slalom_parameters_map(slalom_parameters_map)
                    .angular_velocity_ref(AngularVelocity::new::<radian_per_second>(3.0 * PI))
                    .angular_acceleration_ref(
                        AngularAcceleration::new::<radian_per_second_squared>(36.0 * PI),
                    )
                    .angular_jerk_ref(AngularJerk::new::<radian_per_second_cubed>(1200.0 * PI))
                    .run_slalom_velocity(Velocity::new::<meter_per_second>(1.0))
                    .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(90.0))
                    .spin_angular_acceleration(
                        AngularAcceleration::new::<degree_per_second_squared>(90.0),
                    )
                    .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(180.0))
                    .build::<MathFake, ()>();

                let (kind, last_x, last_y, last_theta) = $value;

                let mut trajectory = generator.generate_search(&(
                    Pose::new(
                        Length::new::<meter>(0.045),
                        Length::new::<meter>(0.09),
                        Angle::new::<degree>(90.0),
                    ),
                    kind,
                ));

                let mut last = trajectory.next().unwrap();
                for target in trajectory {
                    match target {
                        Target::Moving(target) => {
                            let v = target.x.v * target.theta.x.get::<radian>().cos()
                                + target.y.v * target.theta.x.get::<radian>().sin();
                            assert!(
                                v.get::<meter_per_second>()
                                    < search_velocity.get::<meter_per_second>() + EPSILON * 100.0,
                            );
                        }
                        Target::Spin(_) => (),
                    }
                    last = target;
                }
                let last = last.moving().unwrap();
                assert_relative_eq!(last.x.x.get::<meter>(), last_x, epsilon = EPSILON);
                assert_relative_eq!(last.y.x.get::<meter>(), last_y, epsilon = EPSILON);
                assert_relative_eq!(last.theta.x.get::<degree>(), last_theta, epsilon = EPSILON);
            }
        };
    }

    impl_search_trajectory_test!(test_search_trajectory_front: (SearchKind::Front, 0.045, 0.18, 90.0));
    impl_search_trajectory_test!(test_search_trajectory_right: (SearchKind::Right, 0.09, 0.135, 0.0));
    impl_search_trajectory_test!(test_search_trajectory_left: (SearchKind::Left, 0.0, 0.135, 180.0));
    impl_search_trajectory_test!(test_search_trajectory_back: (SearchKind::Back, 0.045, 0.09, 270.0));
}
