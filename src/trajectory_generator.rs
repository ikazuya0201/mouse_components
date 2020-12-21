mod slalom_generator;
mod spin_generator;
mod straight_generator;
mod trajectory;

use core::iter::Chain;

use auto_enums::auto_enum;

use crate::agent::SearchTrajectoryGenerator;
use crate::data_types::Pose;
use crate::maze::RelativeDirection;
use crate::traits::Math;
pub use slalom_generator::{SlalomDirection, SlalomKind, SlalomParameters};
use slalom_generator::{SlalomGenerator, SlalomTrajectory};
use spin_generator::{SpinGenerator, SpinTrajectory};
use straight_generator::{StraightTrajectory, StraightTrajectoryGenerator};
pub use trajectory::{AngleTarget, LengthTarget, Target};
use uom::si::{
    angle::degree,
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, Jerk, Length, Time,
        Velocity,
    },
    length::meter,
};

enum Kind {
    Init,
    Search(RelativeDirection),
}

pub type BackTrajectory = Chain<Chain<StraightTrajectory, SpinTrajectory>, StraightTrajectory>;

pub struct TrajectoryGenerator<M> {
    straight_generator: StraightTrajectoryGenerator<M>,

    #[allow(unused)]
    slalom_generator: SlalomGenerator<M>,
    #[allow(unused)]
    spin_generator: SpinGenerator<M>,

    search_velocity: Velocity,
    front_trajectory: StraightTrajectory,
    right_trajectory: SlalomTrajectory<M>,
    left_trajectory: SlalomTrajectory<M>,
    back_trajectory: BackTrajectory,
}

impl<M> SearchTrajectoryGenerator<Pose, RelativeDirection> for TrajectoryGenerator<M>
where
    M: Math + Clone,
{
    type Target = Target;
    type Trajectory = impl Iterator<Item = Target>;

    fn generate_search_init(&self, pose: &Pose) -> Self::Trajectory {
        self.generate_trajectory(pose, Kind::Init)
    }

    fn generate_search(&self, pose: &Pose, direction: &RelativeDirection) -> Self::Trajectory {
        self.generate_trajectory(pose, Kind::Search(*direction))
    }
}

impl<M> TrajectoryGenerator<M>
where
    M: Math + Clone,
{
    fn get_shift(pose: Pose) -> impl Fn(Target) -> Target {
        let (sin_th, cos_th) = M::sincos(pose.theta);

        move |target: Target| Target {
            x: LengthTarget {
                x: target.x.x * cos_th - target.y.x * sin_th + pose.x,
                v: target.x.v * cos_th - target.y.v * sin_th,
                a: target.x.a * cos_th - target.y.a * sin_th,
                j: target.x.j * cos_th - target.y.j * sin_th,
            },
            y: LengthTarget {
                x: target.x.x * sin_th + target.y.x * cos_th + pose.y,
                v: target.x.v * sin_th + target.y.v * cos_th,
                a: target.x.a * sin_th + target.y.a * cos_th,
                j: target.x.j * sin_th + target.y.j * cos_th,
            },
            theta: AngleTarget {
                x: target.theta.x + pose.theta,
                v: target.theta.v,
                a: target.theta.a,
                j: target.theta.j,
            },
        }
    }

    #[auto_enum]
    fn generate_trajectory(&self, pose: &Pose, kind: Kind) -> impl Iterator<Item = Target> {
        use Kind::*;
        use RelativeDirection::*;

        let shift = Self::get_shift(*pose);
        #[auto_enum(Iterator)]
        match kind {
            Init => {
                let distance = Length::new::<meter>(0.045);
                let (sin_th, cos_th) = M::sincos(pose.theta);
                let x_end = pose.x + distance * cos_th;
                let y_end = pose.y + distance * sin_th;
                self.straight_generator.generate(
                    pose.x,
                    pose.y,
                    x_end,
                    y_end,
                    Default::default(),
                    self.search_velocity,
                )
            }
            Search(direction) => {
                #[auto_enum(Iterator)]
                match direction {
                    Front => self.front_trajectory.clone(),
                    Right => self.right_trajectory.clone(),
                    Left => self.left_trajectory.clone(),
                    Back => self.back_trajectory.clone(),
                    _ => unreachable!(),
                }
            }
            .map(shift),
        }
    }

    pub fn generate_straight(
        &self,
        x_start: Length,
        y_start: Length,
        x_end: Length,
        y_end: Length,
        v_start: Velocity,
        v_end: Velocity,
    ) -> impl Iterator<Item = Target> {
        self.straight_generator
            .generate(x_start, y_start, x_end, y_end, v_start, v_end)
    }

    pub fn generate_spin(
        &self,
        theta_start: Angle,
        theta_distance: Angle,
    ) -> impl Iterator<Item = Target> {
        self.spin_generator.generate(theta_start, theta_distance)
    }
}

pub struct TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, J, T, SV> {
    angular_velocity_ref: TV,
    angular_acceleration_ref: TA,
    angular_jerk_ref: TJ,
    slalom_parameters_map: SPM,
    max_velocity: V,
    max_acceleration: A,
    max_jerk: J,
    period: T,
    search_velocity: SV,
}

impl TrajectoryGeneratorBuilder<(), (), (), (), (), (), (), (), ()> {
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
    >
{
    pub fn build<M: Math>(self) -> TrajectoryGenerator<M> {
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
            let x_end = pose.x + distance * M::cos(pose.theta);
            let y_end = pose.y + distance * M::sin(pose.theta);
            straight_generator.generate(
                pose.x,
                pose.y,
                x_end,
                y_end,
                self.search_velocity,
                self.search_velocity,
            )
        };
        let back_trajectory = {
            let distance = Length::new::<meter>(0.045); //TODO: use configurable value
            let x_end = pose.x + distance * M::cos(pose.theta);
            let y_end = pose.y + distance * M::sin(pose.theta);
            straight_generator
                .generate(
                    pose.x,
                    pose.y,
                    x_end,
                    y_end,
                    self.search_velocity,
                    Default::default(),
                )
                .chain(spin_generator.generate(pose.theta, Angle::new::<degree>(180.0)))
                .chain(straight_generator.generate(
                    x_end,
                    y_end,
                    pose.x,
                    pose.y,
                    Default::default(),
                    self.search_velocity,
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
        }
    }
}

impl<TA, TJ, SPM, V, A, J, T, SV> TrajectoryGeneratorBuilder<(), TA, TJ, SPM, V, A, J, T, SV> {
    pub fn angular_velocity_ref(
        self,
        angular_velocity_ref: AngularVelocity,
    ) -> TrajectoryGeneratorBuilder<AngularVelocity, TA, TJ, SPM, V, A, J, T, SV> {
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
        }
    }
}

impl<TV, TJ, SPM, V, A, J, T, SV> TrajectoryGeneratorBuilder<TV, (), TJ, SPM, V, A, J, T, SV> {
    pub fn angular_acceleration_ref(
        self,
        angular_acceleration_ref: AngularAcceleration,
    ) -> TrajectoryGeneratorBuilder<TV, AngularAcceleration, TJ, SPM, V, A, J, T, SV> {
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
        }
    }
}

impl<TV, TA, SPM, V, A, J, T, SV> TrajectoryGeneratorBuilder<TV, TA, (), SPM, V, A, J, T, SV> {
    pub fn angular_jerk_ref(
        self,
        angular_jerk_ref: AngularJerk,
    ) -> TrajectoryGeneratorBuilder<TV, TA, AngularJerk, SPM, V, A, J, T, SV> {
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
        }
    }
}

impl<TV, TA, TJ, V, A, J, T, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, (), V, A, J, T, SV> {
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
        }
    }
}

impl<TV, TA, TJ, SPM, A, J, T, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, (), A, J, T, SV> {
    pub fn max_velocity(
        self,
        max_velocity: Velocity,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, Velocity, A, J, T, SV> {
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
        }
    }
}

impl<TV, TA, TJ, SPM, V, J, T, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, (), J, T, SV> {
    pub fn max_acceleration(
        self,
        max_acceleration: Acceleration,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, Acceleration, J, T, SV> {
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
        }
    }
}

impl<TV, TA, TJ, SPM, V, A, T, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, (), T, SV> {
    pub fn max_jerk(
        self,
        max_jerk: Jerk,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, Jerk, T, SV> {
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
        }
    }
}

impl<TV, TA, TJ, SPM, V, A, J, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, J, (), SV> {
    pub fn period(
        self,
        period: Time,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, J, Time, SV> {
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
        }
    }
}

impl<TV, TA, TJ, SPM, V, A, J, T> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, J, T, ()> {
    pub fn search_velocity(
        self,
        search_velocity: Velocity,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, SPM, V, A, J, T, Velocity> {
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
        }
    }
}

#[cfg(test)]
pub use slalom_generator::parameters_map;

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

    fn build_generator() -> TrajectoryGenerator<MathFake> {
        TrajectoryGeneratorBuilder::new()
            .max_velocity(Velocity::new::<meter_per_second>(1.0))
            .max_acceleration(Acceleration::new::<meter_per_second_squared>(10.0))
            .max_jerk(Jerk::new::<meter_per_second_cubed>(100.0))
            .angular_velocity_ref(AngularVelocity::new::<degree_per_second>(180.0))
            .angular_acceleration_ref(AngularAcceleration::new::<degree_per_second_squared>(
                1800.0,
            ))
            .angular_jerk_ref(AngularJerk::new::<degree_per_second_cubed>(18000.0))
            .slalom_parameters_map(parameters_map)
            .period(Time::new::<second>(0.001))
            .search_velocity(Velocity::new::<meter_per_second>(0.6))
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
                        .slalom_parameters_map(parameters_map)
                        .angular_velocity_ref(AngularVelocity::new::<radian_per_second>(3.0 * PI))
                        .angular_acceleration_ref(AngularAcceleration::new::<radian_per_second_squared>(
                            36.0 * PI,
                        ))
                        .angular_jerk_ref(AngularJerk::new::<radian_per_second_cubed>(1200.0 * PI))
                        .build::<MathFake>();

                    let (direction, last_x, last_y, last_theta) = $value;

                    let mut trajectory = generator.generate_search(
                        &Pose::new(
                            Length::new::<meter>(0.045),
                            Length::new::<meter>(0.09),
                            Angle::new::<degree>(90.0),
                        ),
                        &direction,
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
