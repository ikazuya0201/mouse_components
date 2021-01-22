mod slalom_generator;
mod spin_generator;
mod straight_generator;
mod trajectory;

use core::iter::Chain;
use core::marker::PhantomData;

use heapless::{ArrayLength, Vec};

use crate::agents::{
    RunTrajectoryGenerator as IRunTrajectoryGenerator,
    SearchTrajectoryGenerator as ISearchTrajectoryGenerator,
};
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

pub struct SearchTrajectoryGenerator<M> {
    initial_trajectory: StraightTrajectory,
    slow_down_trajectory: SlowDownTrajectory,
    front_trajectory: StraightTrajectory,
    right_trajectory: SlalomTrajectory<M>,
    left_trajectory: SlalomTrajectory<M>,
    back_trajectory: BackTrajectory<M>,
}

impl<M: Math> SearchTrajectoryGenerator<M> {
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
        front_offset: Length,
        square_width: Length,
        spin_angular_velocity: AngularVelocity,
        spin_angular_acceleration: AngularAcceleration,
        spin_angular_jerk: AngularJerk,
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
        let spin_generator = SpinGenerator::<M>::new(
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
            initial_trajectory,
            slow_down_trajectory,
            front_trajectory,
            right_trajectory,
            left_trajectory,
            back_trajectory,
        }
    }
}

impl<M> ISearchTrajectoryGenerator<(Pose, SearchKind)> for SearchTrajectoryGenerator<M>
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
impl<M, MaxLength> IRunTrajectoryGenerator<(Pose, RunKind)> for RunTrajectoryGenerator<M, MaxLength>
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

impl<M> SearchTrajectoryGenerator<M>
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

pub struct SearchTrajectoryGeneratorBuilder {
    angular_velocity_ref: Option<AngularVelocity>,
    angular_acceleration_ref: Option<AngularAcceleration>,
    angular_jerk_ref: Option<AngularJerk>,
    slalom_parameters_map: Option<fn(SlalomKind, SlalomDirection) -> SlalomParameters>,
    max_velocity: Option<Velocity>,
    max_acceleration: Option<Acceleration>,
    max_jerk: Option<Jerk>,
    period: Option<Time>,
    search_velocity: Option<Velocity>,
    front_offset: Option<Length>,
    square_width: Option<Length>,
    spin_angular_velocity: Option<AngularVelocity>,
    spin_angular_acceleration: Option<AngularAcceleration>,
    spin_angular_jerk: Option<AngularJerk>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RequiredFieldEmptyError {
    field_name: &'static str,
}

impl core::fmt::Display for RequiredFieldEmptyError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(
            f,
            "Build failed: the field `{}` is required",
            self.field_name
        )
    }
}

fn ok_or<T>(value: Option<T>, field_name: &'static str) -> Result<T, RequiredFieldEmptyError> {
    value.ok_or(RequiredFieldEmptyError { field_name })
}

impl SearchTrajectoryGeneratorBuilder {
    const DEFAULT_FRONT_OFFSET: Length = Length {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.0,
    };
    const DEFAULT_SQUARE_WIDTH: Length = Length {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.09,
    };

    pub fn new() -> Self {
        Self {
            angular_velocity_ref: None,
            angular_acceleration_ref: None,
            angular_jerk_ref: None,
            slalom_parameters_map: None,
            max_velocity: None,
            max_acceleration: None,
            max_jerk: None,
            period: None,
            search_velocity: None,
            front_offset: None,
            square_width: None,
            spin_angular_velocity: None,
            spin_angular_acceleration: None,
            spin_angular_jerk: None,
        }
    }

    pub fn build<M: Math>(self) -> Result<SearchTrajectoryGenerator<M>, RequiredFieldEmptyError> {
        Ok(SearchTrajectoryGenerator::<M>::new(
            ok_or(self.angular_velocity_ref, "angular_velocity_ref")?,
            ok_or(self.angular_acceleration_ref, "angular_acceleration_ref")?,
            ok_or(self.angular_jerk_ref, "angular_jerk_ref")?,
            ok_or(self.slalom_parameters_map, "slalom_parameters_map")?,
            ok_or(self.max_velocity, "max_velocity")?,
            ok_or(self.max_acceleration, "max_acceleration")?,
            ok_or(self.max_jerk, "max_jerk")?,
            ok_or(self.period, "period")?,
            ok_or(self.search_velocity, "search_velocity")?,
            self.front_offset.unwrap_or(Self::DEFAULT_FRONT_OFFSET),
            self.square_width.unwrap_or(Self::DEFAULT_SQUARE_WIDTH),
            ok_or(self.spin_angular_velocity, "spin_angular_velocity")?,
            ok_or(self.spin_angular_acceleration, "spin_angular_acceleration")?,
            ok_or(self.spin_angular_jerk, "spin_angular_jerk")?,
        ))
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

    pub fn search_velocity(mut self, search_velocity: Velocity) -> Self {
        self.search_velocity = Some(search_velocity);
        self
    }

    pub fn front_offset(mut self, front_offset: Length) -> Self {
        self.front_offset = Some(front_offset);
        self
    }

    pub fn square_width(mut self, square_width: Length) -> Self {
        self.square_width = Some(square_width);
        self
    }

    pub fn spin_angular_velocity(mut self, spin_angular_velocity: AngularVelocity) -> Self {
        self.spin_angular_velocity = Some(spin_angular_velocity);
        self
    }

    pub fn spin_angular_acceleration(
        mut self,
        spin_angular_acceleration: AngularAcceleration,
    ) -> Self {
        self.spin_angular_acceleration = Some(spin_angular_acceleration);
        self
    }

    pub fn spin_angular_jerk(mut self, spin_angular_jerk: AngularJerk) -> Self {
        self.spin_angular_jerk = Some(spin_angular_jerk);
        self
    }
}

pub struct RunTrajectoryGeneratorBuilder {
    run_slalom_velocity: Option<Velocity>,
    max_velocity: Option<Velocity>,
    max_acceleration: Option<Acceleration>,
    max_jerk: Option<Jerk>,
    angular_velocity_ref: Option<AngularVelocity>,
    angular_acceleration_ref: Option<AngularAcceleration>,
    angular_jerk_ref: Option<AngularJerk>,
    slalom_parameters_map: Option<fn(SlalomKind, SlalomDirection) -> SlalomParameters>,
    period: Option<Time>,
}

impl RunTrajectoryGeneratorBuilder {
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
        }
    }

    pub fn build<M: Math, MaxLength>(
        self,
    ) -> Result<RunTrajectoryGenerator<M, MaxLength>, RequiredFieldEmptyError> {
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

    fn build_generator() -> SearchTrajectoryGenerator<MathFake> {
        SearchTrajectoryGeneratorBuilder::new()
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
            .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(90.0))
            .spin_angular_acceleration(AngularAcceleration::new::<degree_per_second_squared>(90.0))
            .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(180.0))
            .build()
            .unwrap()
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

                let generator = SearchTrajectoryGeneratorBuilder::new()
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
                    .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(90.0))
                    .spin_angular_acceleration(
                        AngularAcceleration::new::<degree_per_second_squared>(90.0),
                    )
                    .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(180.0))
                    .build::<MathFake>()
                    .unwrap();

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
