mod slalom_generator;
mod spin_generator;
mod straight_generator;
mod trajectory;

use core::iter::Chain;

use auto_enums::auto_enum;

use super::agent;
use crate::agent::pose::Pose;
use crate::maze::RelativeDirection;
use slalom_generator::{SlalomGenerator, SlalomTrajectory};
use spin_generator::{SpinGenerator, SpinTrajectory};
use straight_generator::{StraightTrajectory, StraightTrajectoryGenerator};
pub use trajectory::{AngleTarget, LengthTarget, Target};
use uom::si::{
    angle::{degree, radian},
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

pub struct TrajectoryGenerator {
    straight_generator: StraightTrajectoryGenerator,

    #[allow(unused)]
    slalom_generator: SlalomGenerator,
    #[allow(unused)]
    spin_generator: SpinGenerator,

    search_velocity: Velocity,
    front_trajectory: StraightTrajectory,
    right_trajectory: SlalomTrajectory,
    left_trajectory: SlalomTrajectory,
    back_trajectory: BackTrajectory,
}

impl agent::TrajectoryGenerator<Pose, Target, RelativeDirection> for TrajectoryGenerator {
    type Trajectory = impl Iterator<Item = Target>;

    fn generate_search_init(&self, pose: Pose) -> Self::Trajectory {
        self.generate_trajectory(pose, Kind::Init)
    }

    fn generate_search(&self, pose: Pose, direction: RelativeDirection) -> Self::Trajectory {
        self.generate_trajectory(pose, Kind::Search(direction))
    }
}

impl TrajectoryGenerator {
    fn get_shift(pose: Pose) -> impl Fn(Target) -> Target {
        let cos_th = libm::cosf(pose.theta.get::<radian>());
        let sin_th = libm::sinf(pose.theta.get::<radian>());

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
    fn generate_trajectory(&self, pose: Pose, kind: Kind) -> impl Iterator<Item = Target> {
        use Kind::*;
        use RelativeDirection::*;

        let shift = Self::get_shift(pose);
        #[auto_enum(Iterator)]
        match kind {
            Init => {
                let distance = Length::new::<meter>(0.045);
                let x_end = pose.x + distance * libm::cosf(pose.theta.get::<radian>());
                let y_end = pose.y + distance * libm::sinf(pose.theta.get::<radian>());
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

    #[cfg(any(test, feature = "debug"))]
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

    #[cfg(any(test, feature = "debug"))]
    pub fn generate_spin(
        &self,
        theta_start: Angle,
        theta_distance: Angle,
    ) -> impl Iterator<Item = Target> {
        self.spin_generator.generate(theta_start, theta_distance)
    }
}

pub struct TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, J, T, SV> {
    angular_velocity_ref: TV,
    angular_acceleration_ref: TA,
    angular_jerk_ref: TJ,
    slalom_velocity_ref: VR,
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
            slalom_velocity_ref: (),
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
        Velocity,
        Velocity,
        Acceleration,
        Jerk,
        Time,
        Velocity,
    >
{
    pub fn build(self) -> TrajectoryGenerator {
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
            self.slalom_velocity_ref,
            self.period,
        );
        let spin_generator = SpinGenerator::new(
            self.angular_velocity_ref,
            self.angular_acceleration_ref,
            self.angular_jerk_ref,
            self.period,
        );

        let pose = Pose::default();
        let right_trajectory = slalom_generator.generate(
            pose.x,
            pose.y,
            pose.theta,
            Angle::new::<degree>(-90.0),
            self.search_velocity,
        );
        let left_trajectory = slalom_generator.generate(
            pose.x,
            pose.y,
            pose.theta,
            Angle::new::<degree>(90.0),
            self.search_velocity,
        );
        let front_trajectory = {
            let distance = Length::new::<meter>(0.09); //TODO: use configurable value
            let x_end = pose.x + distance * libm::cosf(pose.theta.get::<radian>());
            let y_end = pose.y + distance * libm::sinf(pose.theta.get::<radian>());
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
            let x_end = pose.x + distance * libm::cosf(pose.theta.get::<radian>());
            let y_end = pose.y + distance * libm::sinf(pose.theta.get::<radian>());
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

impl<TA, TJ, VR, V, A, J, T, SV> TrajectoryGeneratorBuilder<(), TA, TJ, VR, V, A, J, T, SV> {
    pub fn angular_velocity_ref(
        self,
        angular_velocity_ref: AngularVelocity,
    ) -> TrajectoryGeneratorBuilder<AngularVelocity, TA, TJ, VR, V, A, J, T, SV> {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_velocity_ref: self.slalom_velocity_ref,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
        }
    }
}

impl<TV, TJ, VR, V, A, J, T, SV> TrajectoryGeneratorBuilder<TV, (), TJ, VR, V, A, J, T, SV> {
    pub fn angular_acceleration_ref(
        self,
        angular_acceleration_ref: AngularAcceleration,
    ) -> TrajectoryGeneratorBuilder<TV, AngularAcceleration, TJ, VR, V, A, J, T, SV> {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_velocity_ref: self.slalom_velocity_ref,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
        }
    }
}

impl<TV, TA, VR, V, A, J, T, SV> TrajectoryGeneratorBuilder<TV, TA, (), VR, V, A, J, T, SV> {
    pub fn angular_jerk_ref(
        self,
        angular_jerk_ref: AngularJerk,
    ) -> TrajectoryGeneratorBuilder<TV, TA, AngularJerk, VR, V, A, J, T, SV> {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref,
            slalom_velocity_ref: self.slalom_velocity_ref,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
        }
    }
}

impl<TV, TA, TJ, V, A, J, T, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, (), V, A, J, T, SV> {
    pub fn slalom_velocity_ref(
        self,
        slalom_velocity_ref: Velocity,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, Velocity, V, A, J, T, SV> {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_velocity_ref,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
        }
    }
}

impl<TV, TA, TJ, VR, A, J, T, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, (), A, J, T, SV> {
    pub fn max_velocity(
        self,
        max_velocity: Velocity,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, Velocity, A, J, T, SV> {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_velocity_ref: self.slalom_velocity_ref,
            max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
        }
    }
}

impl<TV, TA, TJ, VR, V, J, T, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, (), J, T, SV> {
    pub fn max_acceleration(
        self,
        max_acceleration: Acceleration,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, Acceleration, J, T, SV> {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_velocity_ref: self.slalom_velocity_ref,
            max_velocity: self.max_velocity,
            max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
        }
    }
}

impl<TV, TA, TJ, VR, V, A, T, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, (), T, SV> {
    pub fn max_jerk(
        self,
        max_jerk: Jerk,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, Jerk, T, SV> {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_velocity_ref: self.slalom_velocity_ref,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk,
            period: self.period,
            search_velocity: self.search_velocity,
        }
    }
}

impl<TV, TA, TJ, VR, V, A, J, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, J, (), SV> {
    pub fn period(
        self,
        period: Time,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, J, Time, SV> {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_velocity_ref: self.slalom_velocity_ref,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period,
            search_velocity: self.search_velocity,
        }
    }
}

impl<TV, TA, TJ, VR, V, A, J, T> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, J, T, ()> {
    pub fn search_velocity(
        self,
        search_velocity: Velocity,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, J, T, Velocity> {
        TrajectoryGeneratorBuilder {
            angular_velocity_ref: self.angular_velocity_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_velocity_ref: self.slalom_velocity_ref,
            max_velocity: self.max_velocity,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_velocity,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use uom::si::{
        acceleration::meter_per_second_squared,
        angular_acceleration::{degree_per_second_squared, radian_per_second_squared},
        angular_jerk::{degree_per_second_cubed, radian_per_second_cubed},
        angular_velocity::{degree_per_second, radian_per_second},
        jerk::meter_per_second_cubed,
        time::second,
        velocity::meter_per_second,
    };

    fn build_generator() -> TrajectoryGenerator {
        TrajectoryGeneratorBuilder::new()
            .max_velocity(Velocity::new::<meter_per_second>(1.0))
            .max_acceleration(Acceleration::new::<meter_per_second_squared>(10.0))
            .max_jerk(Jerk::new::<meter_per_second_cubed>(100.0))
            .angular_velocity_ref(AngularVelocity::new::<degree_per_second>(180.0))
            .angular_acceleration_ref(AngularAcceleration::new::<degree_per_second_squared>(
                1800.0,
            ))
            .angular_jerk_ref(AngularJerk::new::<degree_per_second_cubed>(18000.0))
            .slalom_velocity_ref(Velocity::new::<meter_per_second>(0.6))
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
                    use agent::TrajectoryGenerator;
                    use core::f32::consts::PI;

                    const EPSILON: f32 = 2e-4;

                    let period = Time::new::<second>(0.001);
                    let search_velocity = Velocity::new::<meter_per_second>(0.2);

                    let generator = TrajectoryGeneratorBuilder::new()
                        .period(period)
                        .max_velocity(Velocity::new::<meter_per_second>(2.0))
                        .max_acceleration(Acceleration::new::<meter_per_second_squared>(0.7))
                        .max_jerk(Jerk::new::<meter_per_second_cubed>(1.0))
                        .search_velocity(search_velocity)
                        .slalom_velocity_ref(Velocity::new::<meter_per_second>(0.27178875))
                        .angular_velocity_ref(AngularVelocity::new::<radian_per_second>(3.0 * PI))
                        .angular_acceleration_ref(AngularAcceleration::new::<radian_per_second_squared>(
                            36.0 * PI,
                        ))
                        .angular_jerk_ref(AngularJerk::new::<radian_per_second_cubed>(1200.0 * PI))
                        .build();

                    let (direction, last_x, last_y, last_theta) = $value;

                    let mut trajectory = generator.generate_search(
                        Pose::new(
                            Length::new::<meter>(0.045),
                            Length::new::<meter>(0.09),
                            Angle::new::<degree>(90.0),
                        ),
                        direction,
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
