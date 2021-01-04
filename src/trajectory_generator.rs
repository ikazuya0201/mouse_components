mod slalom_generator;
mod spin_generator;
mod straight_generator;
mod trajectory;

use core::iter::Chain;
use core::marker::PhantomData;

use heapless::{ArrayLength, Vec};

use crate::agent::{RunTrajectoryGenerator, SearchTrajectoryGenerator};
use crate::data_types::Pose;
use crate::maze::RelativeDirection;
use crate::traits::Math;
pub use slalom_generator::{slalom_parameters_map, SlalomDirection, SlalomKind, SlalomParameters};
use slalom_generator::{SlalomGenerator, SlalomTrajectory};
use spin_generator::{SpinGenerator, SpinTrajectory};
use straight_generator::{StraightTrajectory, StraightTrajectoryGenerator};
pub use trajectory::{AngleTarget, LengthTarget, ShiftTrajectory, Target};
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
    Search(RelativeDirection),
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RunKind {
    Straight(u16),
    StraightDiagonal(u16),
    Slalom(SlalomKind, SlalomDirection),
}

pub enum SearchTrajectory<M> {
    Init(StraightTrajectory),
    Right(SlalomTrajectory<M>),
    Left(SlalomTrajectory<M>),
    Front(StraightTrajectory),
    Back(BackTrajectory<M>),
}

impl<M: Math> Iterator for SearchTrajectory<M> {
    type Item = Target;

    fn next(&mut self) -> Option<Self::Item> {
        use SearchTrajectory::*;

        match self {
            Init(inner) => inner.next(),
            Right(inner) => inner.next(),
            Left(inner) => inner.next(),
            Front(inner) => inner.next(),
            Back(inner) => inner.next(),
        }
    }
}

pub type BackTrajectory<M> =
    Chain<Chain<StraightTrajectory, SpinTrajectory>, ShiftTrajectory<StraightTrajectory, M>>;

pub struct TrajectoryGenerator<M, MaxLength> {
    straight_generator: StraightTrajectoryGenerator<M>,

    #[allow(unused)]
    slalom_generator: SlalomGenerator<M>,
    #[allow(unused)]
    spin_generator: SpinGenerator<M>,

    search_velocity: Velocity,
    run_slalom_velocity: Velocity,

    front_trajectory: StraightTrajectory,
    right_trajectory: SlalomTrajectory<M>,
    left_trajectory: SlalomTrajectory<M>,
    back_trajectory: BackTrajectory<M>,

    _max_length: PhantomData<fn() -> MaxLength>,
}

impl<M, MaxLength> SearchTrajectoryGenerator<Pose, SearchKind> for TrajectoryGenerator<M, MaxLength>
where
    M: Math,
{
    type Target = Target;
    type Trajectory = ShiftTrajectory<SearchTrajectory<M>, M>;

    fn generate_search(&self, pose: &Pose, kind: &SearchKind) -> Self::Trajectory {
        self.generate_search_trajectory(pose, kind)
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
        use RelativeDirection::*;
        use SearchKind::*;

        ShiftTrajectory::<_, M>::new(
            *pose,
            match kind {
                Init => {
                    let distance = Length::new::<meter>(0.045);
                    SearchTrajectory::Init(self.straight_generator.generate(
                        distance,
                        Default::default(),
                        self.search_velocity,
                    ))
                }
                Search(direction) => match direction {
                    Front => SearchTrajectory::Front(self.front_trajectory.clone()),
                    Right => SearchTrajectory::Right(self.right_trajectory.clone()),
                    Left => SearchTrajectory::Left(self.left_trajectory.clone()),
                    Back => SearchTrajectory::Back(self.back_trajectory.clone()),
                    _ => unreachable!(),
                },
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
                let distance = *len as f32 * Length::new::<meter>(0.045);
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

pub struct TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, J, T, SV, RunSlalomVelocity> {
    angular_velocity_ref: TV,
    angular_acceleration_ref: TA,
    angular_jerk_ref: TJ,
    slalom_parameters_map: SPM,
    max_velocity: V,
    max_acceleration: A,
    max_jerk: J,
    period: T,
    search_velocity: SV,
    run_slalom_velocity: RunSlalomVelocity,
}

impl TrajectoryGeneratorBuilder<(), (), (), (), (), (), (), (), (), ()> {
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
    >
{
    pub fn build<M: Math, MaxLength>(self) -> TrajectoryGenerator<M, MaxLength> {
        let straight_generator = StraightTrajectoryGenerator::new(
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.period,
        );
        let slalom_generator = SlalomGenerator::new(
            self.angular_velocity_ref,
            self.angular_acceleration_ref,
            self.angular_jerk_ref,
            self.period,
            self.slalom_parameters_map,
        );
        let spin_generator = SpinGenerator::new(
            self.angular_velocity_ref,
            self.angular_acceleration_ref,
            self.angular_jerk_ref,
            self.period,
        );

        let pose = Pose::default();
        let right_trajectory = slalom_generator.generate_slalom(
            SlalomKind::Search90,
            SlalomDirection::Right,
            self.search_velocity,
        );
        let left_trajectory = slalom_generator.generate_slalom(
            SlalomKind::Search90,
            SlalomDirection::Left,
            self.search_velocity,
        );
        let front_trajectory = {
            let distance = Length::new::<meter>(0.09); //TODO: use configurable value
            straight_generator.generate(distance, self.search_velocity, self.search_velocity)
        };
        let back_trajectory = {
            let distance = Length::new::<meter>(0.045); //TODO: use configurable value
            straight_generator
                .generate(distance, self.search_velocity, Default::default())
                .chain(spin_generator.generate(pose.theta, Angle::new::<degree>(180.0)))
                .chain(ShiftTrajectory::new(
                    Pose {
                        x: distance,
                        y: Default::default(),
                        theta: Angle::new::<degree>(180.0),
                    },
                    straight_generator.generate(distance, Default::default(), self.search_velocity),
                ))
        };

        TrajectoryGenerator {
            straight_generator,
            slalom_generator,
            spin_generator,
            search_velocity: self.search_velocity,
            front_trajectory,
            right_trajectory,
            left_trajectory,
            back_trajectory,
            run_slalom_velocity: self.run_slalom_velocity,
            _max_length: PhantomData,
        }
    }
}

impl<TA, TJ, SPM, V, A, J, T, SV, RunSlalomVelocity>
    TrajectoryGeneratorBuilder<(), TA, TJ, SPM, V, A, J, T, SV, RunSlalomVelocity>
{
    pub fn angular_velocity_ref(
        self,
        angular_velocity_ref: AngularVelocity,
    ) -> TrajectoryGeneratorBuilder<AngularVelocity, TA, TJ, SPM, V, A, J, T, SV, RunSlalomVelocity>
    {
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
        }
    }
}

impl<TV, TJ, SPM, V, A, J, T, SV, RunSlalomVelocity>
    TrajectoryGeneratorBuilder<TV, (), TJ, SPM, V, A, J, T, SV, RunSlalomVelocity>
{
    pub fn angular_acceleration_ref(
        self,
        angular_acceleration_ref: AngularAcceleration,
    ) -> TrajectoryGeneratorBuilder<
        TV,
        AngularAcceleration,
        TJ,
        SPM,
        V,
        A,
        J,
        T,
        SV,
        RunSlalomVelocity,
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
        }
    }
}

impl<TV, TA, SPM, V, A, J, T, SV, RunSlalomVelocity>
    TrajectoryGeneratorBuilder<TV, TA, (), SPM, V, A, J, T, SV, RunSlalomVelocity>
{
    pub fn angular_jerk_ref(
        self,
        angular_jerk_ref: AngularJerk,
    ) -> TrajectoryGeneratorBuilder<TV, TA, AngularJerk, SPM, V, A, J, T, SV, RunSlalomVelocity>
    {
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
        }
    }
}

impl<TV, TA, TJ, V, A, J, T, SV, RunSlalomVelocity>
    TrajectoryGeneratorBuilder<TV, TA, TJ, (), V, A, J, T, SV, RunSlalomVelocity>
{
    pub fn slalom_parameters_map(
        self,
        slalom_parameters_map: fn(SlalomKind, SlalomDirection) -> SlalomParameters,
    ) -> TrajectoryGeneratorBuilder<
        TV,
        TA,
        TJ,
        fn(SlalomKind, SlalomDirection) -> SlalomParameters,
        V,
        A,
        J,
        T,
        SV,
        RunSlalomVelocity,
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
        }
    }
}

impl<TV, TA, TJ, SPM, A, J, T, SV, RunSlalomVelocity>
    TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, (), A, J, T, SV, RunSlalomVelocity>
{
    pub fn max_velocity(
        self,
        max_velocity: Velocity,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, Velocity, A, J, T, SV, RunSlalomVelocity> {
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
        }
    }
}

impl<TV, TA, TJ, SPM, V, J, T, SV, RunSlalomVelocity>
    TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, (), J, T, SV, RunSlalomVelocity>
{
    pub fn max_acceleration(
        self,
        max_acceleration: Acceleration,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, Acceleration, J, T, SV, RunSlalomVelocity>
    {
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
        }
    }
}

impl<TV, TA, TJ, SPM, V, A, T, SV, RunSlalomVelocity>
    TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, (), T, SV, RunSlalomVelocity>
{
    pub fn max_jerk(
        self,
        max_jerk: Jerk,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, Jerk, T, SV, RunSlalomVelocity> {
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
        }
    }
}

impl<TV, TA, TJ, SPM, V, A, J, SV, RunSlalomVelocity>
    TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, J, (), SV, RunSlalomVelocity>
{
    pub fn period(
        self,
        period: Time,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, J, Time, SV, RunSlalomVelocity> {
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
        }
    }
}

impl<TV, TA, TJ, SPM, V, A, J, T, RunSlalomVelocity>
    TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, J, T, (), RunSlalomVelocity>
{
    pub fn search_velocity(
        self,
        search_velocity: Velocity,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, J, T, Velocity, RunSlalomVelocity> {
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
        }
    }
}

impl<TV, TA, TJ, SPM, V, A, J, T, SV>
    TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, J, T, SV, ()>
{
    pub fn run_slalom_velocity(
        self,
        run_slalom_velocity: Velocity,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, J, T, SV, Velocity> {
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
            .build()
    }

    #[test]
    fn test_build() {
        build_generator();
    }

    macro_rules! search_trajectory_tests {
        ($($name: ident: $value: expr,)*) => {
            $(
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
                        .angular_acceleration_ref(AngularAcceleration::new::<radian_per_second_squared>(
                            36.0 * PI,
                        ))
                        .angular_jerk_ref(AngularJerk::new::<radian_per_second_cubed>(1200.0 * PI))
                        .run_slalom_velocity(Velocity::new::<meter_per_second>(1.0))
                        .build::<MathFake, ()>();

                    let (direction, last_x, last_y, last_theta) = $value;

                    let mut trajectory = generator.generate_search(
                        &Pose::new(
                            Length::new::<meter>(0.045),
                            Length::new::<meter>(0.09),
                            Angle::new::<degree>(90.0),
                        ),
                        &SearchKind::Search(direction),
                    );

                    let mut last = trajectory.next().unwrap();
                    for target in trajectory {
                        let v = target.x.v * target.theta.x.get::<radian>().cos()
                            + target.y.v * target.theta.x.get::<radian>().sin();
                        assert!(
                            v.get::<meter_per_second>() < search_velocity.get::<meter_per_second>() + EPSILON * 100.0,
                        );
                        last = target;
                    }
                    assert_relative_eq!(last.x.x.get::<meter>(), last_x, epsilon = EPSILON);
                    assert_relative_eq!(last.y.x.get::<meter>(), last_y, epsilon = EPSILON);
                    assert_relative_eq!(last.theta.x.get::<degree>(), last_theta, epsilon = EPSILON);
                }
            )*
        }
    }

    search_trajectory_tests! {
        test_search_trajectory_front: (RelativeDirection::Front, 0.045, 0.18, 90.0),
        test_search_trajectory_right: (RelativeDirection::Right, 0.09, 0.135, 0.0),
        test_search_trajectory_left: (RelativeDirection::Left, 0.0, 0.135, 180.0),
        test_search_trajectory_back: (RelativeDirection::Back, 0.045, 0.09, 270.0),
    }
}
