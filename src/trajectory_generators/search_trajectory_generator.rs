use core::iter::Chain;
use core::marker::PhantomData;

use serde::{Deserialize, Serialize};

use super::slalom_generator::{SlalomDirection, SlalomKind, SlalomParametersGenerator};
use super::slalom_generator::{SlalomGenerator, SlalomTrajectory};
use super::spin_generator::{SpinGenerator, SpinTrajectory};
use super::straight_generator::{StraightTrajectory, StraightTrajectoryGenerator};
use super::trajectory::StopTrajectory;
use super::trajectory::{ShiftTrajectory, Target};
use super::Pose;
use crate::trajectory_managers::SearchTrajectoryGenerator as ISearchTrajectoryGenerator;
use crate::utils::builder::RequiredFieldEmptyError;
use crate::utils::math::Math;
use crate::{get_or_err, impl_setter};
use uom::si::{
    angle::degree,
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, Jerk, Length, Time,
        Velocity,
    },
};

#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum SearchKind {
    Init,
    Final,
    Front,
    Right,
    Left,
    Back,
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

impl<M> SearchTrajectoryGenerator<M>
where
    M: Math,
{
    fn new<Generator>(
        parameters_generator: Generator,
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
    ) -> Self
    where
        Generator: SlalomParametersGenerator,
    {
        let straight_generator =
            StraightTrajectoryGenerator::new(max_velocity, max_acceleration, max_jerk, period);
        let slalom_generator = SlalomGenerator::new(
            period,
            parameters_generator,
            max_velocity,
            max_acceleration,
            max_jerk,
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
        let right_trajectory = slalom_generator.generate_constant_slalom(
            SlalomKind::Search90,
            SlalomDirection::Right,
            search_velocity,
        );
        let left_trajectory = slalom_generator.generate_constant_slalom(
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

pub struct SearchTrajectoryGeneratorBuilder<M, Generator> {
    parameters_generator: Option<Generator>,
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
    _math: PhantomData<fn() -> M>,
}

impl<Generator> Default
    for SearchTrajectoryGeneratorBuilder<crate::utils::math::LibmMath, Generator>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<M, Generator> SearchTrajectoryGeneratorBuilder<M, Generator> {
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
            parameters_generator: None,
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
            _math: PhantomData,
        }
    }

    pub fn build(&mut self) -> Result<SearchTrajectoryGenerator<M>, RequiredFieldEmptyError>
    where
        M: Math,
        Generator: SlalomParametersGenerator,
    {
        Ok(SearchTrajectoryGenerator::new(
            get_or_err!(self.parameters_generator),
            get_or_err!(self.max_velocity),
            get_or_err!(self.max_acceleration),
            get_or_err!(self.max_jerk),
            get_or_err!(self.period),
            get_or_err!(self.search_velocity),
            self.front_offset.unwrap_or(Self::DEFAULT_FRONT_OFFSET),
            self.square_width.unwrap_or(Self::DEFAULT_SQUARE_WIDTH),
            get_or_err!(self.spin_angular_velocity),
            get_or_err!(self.spin_angular_acceleration),
            get_or_err!(self.spin_angular_jerk),
        ))
    }

    impl_setter!(parameters_generator: Generator);
    impl_setter!(max_velocity: Velocity);
    impl_setter!(max_acceleration: Acceleration);
    impl_setter!(max_jerk: Jerk);
    impl_setter!(period: Time);
    impl_setter!(search_velocity: Velocity);
    impl_setter!(front_offset: Length);
    impl_setter!(square_width: Length);
    impl_setter!(spin_angular_velocity: AngularVelocity);
    impl_setter!(spin_angular_acceleration: AngularAcceleration);
    impl_setter!(spin_angular_jerk: AngularJerk);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trajectory_generators::DefaultSlalomParametersGenerator;
    use crate::utils::math::MathFake;
    use approx::assert_relative_eq;
    use uom::si::{
        acceleration::meter_per_second_squared, angle::radian,
        angular_acceleration::degree_per_second_squared, angular_jerk::degree_per_second_cubed,
        angular_velocity::degree_per_second, jerk::meter_per_second_cubed, length::meter,
        time::second, velocity::meter_per_second,
    };

    fn build_generator() -> SearchTrajectoryGenerator<MathFake> {
        SearchTrajectoryGeneratorBuilder::new()
            .max_velocity(Velocity::new::<meter_per_second>(1.0))
            .max_acceleration(Acceleration::new::<meter_per_second_squared>(10.0))
            .max_jerk(Jerk::new::<meter_per_second_cubed>(100.0))
            .parameters_generator(DefaultSlalomParametersGenerator)
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
                use crate::trajectory_generators::DefaultSlalomParametersGenerator;
                use crate::utils::math::MathFake;

                const EPSILON: f32 = 1e-3;

                let period = Time::new::<second>(0.001);
                let search_velocity = Velocity::new::<meter_per_second>(0.2);

                let generator = SearchTrajectoryGeneratorBuilder::<MathFake, _>::new()
                    .period(period)
                    .max_velocity(Velocity::new::<meter_per_second>(2.0))
                    .max_acceleration(Acceleration::new::<meter_per_second_squared>(0.7))
                    .max_jerk(Jerk::new::<meter_per_second_cubed>(1.0))
                    .search_velocity(search_velocity)
                    .parameters_generator(DefaultSlalomParametersGenerator)
                    .spin_angular_velocity(AngularVelocity::new::<degree_per_second>(90.0))
                    .spin_angular_acceleration(
                        AngularAcceleration::new::<degree_per_second_squared>(90.0),
                    )
                    .spin_angular_jerk(AngularJerk::new::<degree_per_second_cubed>(180.0))
                    .build()
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
