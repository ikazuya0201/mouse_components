mod slalom_generator;
mod spin_generator;
mod straight_generator;
mod trajectory;

use core::iter::Chain;

use auto_enums::auto_enum;
use quantities::{
    Acceleration, Angle, AngularAcceleration, AngularJerk, AngularSpeed, Distance, Jerk, Speed,
    Time,
};

use super::agent;
use crate::agent::pose::Pose;
use crate::maze::RelativeDirection;
use slalom_generator::{SlalomGenerator, SlalomTrajectory};
use spin_generator::{SpinGenerator, SpinTrajectory};
use straight_generator::{StraightTrajectory, StraightTrajectoryGenerator};
pub use trajectory::{SubTarget, Target};

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

    search_speed: Speed,
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
        let cos_th = pose.theta.cos();
        let sin_th = pose.theta.sin();

        move |target: Target| Target {
            x: SubTarget {
                x: target.x.x * cos_th - target.y.x * sin_th + pose.x,
                v: target.x.v * cos_th - target.y.v * sin_th,
                a: target.x.a * cos_th - target.y.a * sin_th,
                j: target.x.j * cos_th - target.y.j * sin_th,
            },
            y: SubTarget {
                x: target.x.x * sin_th + target.y.x * cos_th + pose.y,
                v: target.x.v * sin_th + target.y.v * cos_th,
                a: target.x.a * sin_th + target.y.a * cos_th,
                j: target.x.j * sin_th + target.y.j * cos_th,
            },
            theta: SubTarget {
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
                let distance = Distance::from_meters(0.045);
                let x_end = pose.x + distance * pose.theta.cos();
                let y_end = pose.y + distance * pose.theta.sin();
                self.straight_generator.generate(
                    pose.x,
                    pose.y,
                    x_end,
                    y_end,
                    Default::default(),
                    self.search_speed,
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
        x_start: Distance,
        y_start: Distance,
        x_end: Distance,
        y_end: Distance,
        v_start: Speed,
        v_end: Speed,
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
    angular_speed_ref: TV,
    angular_acceleration_ref: TA,
    angular_jerk_ref: TJ,
    slalom_speed_ref: VR,
    max_speed: V,
    max_acceleration: A,
    max_jerk: J,
    period: T,
    search_speed: SV,
}

impl TrajectoryGeneratorBuilder<(), (), (), (), (), (), (), (), ()> {
    pub fn new() -> Self {
        Self {
            angular_speed_ref: (),
            angular_acceleration_ref: (),
            angular_jerk_ref: (),
            slalom_speed_ref: (),
            max_speed: (),
            max_acceleration: (),
            max_jerk: (),
            period: (),
            search_speed: (),
        }
    }
}

impl
    TrajectoryGeneratorBuilder<
        AngularSpeed,
        AngularAcceleration,
        AngularJerk,
        Speed,
        Speed,
        Acceleration,
        Jerk,
        Time,
        Speed,
    >
{
    pub fn build(self) -> TrajectoryGenerator {
        let straight_generator = StraightTrajectoryGenerator::new(
            self.max_speed,
            self.max_acceleration,
            self.max_jerk,
            self.period,
        );
        let slalom_generator = SlalomGenerator::new(
            self.angular_speed_ref,
            self.angular_acceleration_ref,
            self.angular_jerk_ref,
            self.slalom_speed_ref,
            self.period,
        );
        let spin_generator = SpinGenerator::new(
            self.angular_speed_ref,
            self.angular_acceleration_ref,
            self.angular_jerk_ref,
            self.period,
        );

        let pose = Pose::default();
        let right_trajectory = slalom_generator.generate(
            pose.x,
            pose.y,
            pose.theta,
            Angle::from_degree(-90.0),
            self.search_speed,
        );
        let left_trajectory = slalom_generator.generate(
            pose.x,
            pose.y,
            pose.theta,
            Angle::from_degree(90.0),
            self.search_speed,
        );
        let front_trajectory = {
            let distance = Distance::from_meters(0.09); //TODO: use configurable value
            let x_end = pose.x + distance * pose.theta.cos();
            let y_end = pose.y + distance * pose.theta.sin();
            straight_generator.generate(
                pose.x,
                pose.y,
                x_end,
                y_end,
                self.search_speed,
                self.search_speed,
            )
        };
        let back_trajectory = {
            let distance = Distance::from_meters(0.045); //TODO: use configurable value
            let x_end = pose.x + distance * pose.theta.cos();
            let y_end = pose.y + distance * pose.theta.sin();
            straight_generator
                .generate(
                    pose.x,
                    pose.y,
                    x_end,
                    y_end,
                    self.search_speed,
                    Default::default(),
                )
                .chain(spin_generator.generate(pose.theta, Angle::from_degree(180.0)))
                .chain(straight_generator.generate(
                    x_end,
                    y_end,
                    pose.x,
                    pose.y,
                    Default::default(),
                    self.search_speed,
                ))
        };

        TrajectoryGenerator {
            straight_generator,
            slalom_generator,
            spin_generator,
            search_speed: self.search_speed,
            front_trajectory,
            right_trajectory,
            left_trajectory,
            back_trajectory,
        }
    }
}

impl<TA, TJ, VR, V, A, J, T, SV> TrajectoryGeneratorBuilder<(), TA, TJ, VR, V, A, J, T, SV> {
    pub fn angular_speed_ref(
        self,
        angular_speed_ref: AngularSpeed,
    ) -> TrajectoryGeneratorBuilder<AngularSpeed, TA, TJ, VR, V, A, J, T, SV> {
        TrajectoryGeneratorBuilder {
            angular_speed_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_speed_ref: self.slalom_speed_ref,
            max_speed: self.max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_speed: self.search_speed,
        }
    }
}

impl<TV, TJ, VR, V, A, J, T, SV> TrajectoryGeneratorBuilder<TV, (), TJ, VR, V, A, J, T, SV> {
    pub fn angular_acceleration_ref(
        self,
        angular_acceleration_ref: AngularAcceleration,
    ) -> TrajectoryGeneratorBuilder<TV, AngularAcceleration, TJ, VR, V, A, J, T, SV> {
        TrajectoryGeneratorBuilder {
            angular_speed_ref: self.angular_speed_ref,
            angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_speed_ref: self.slalom_speed_ref,
            max_speed: self.max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_speed: self.search_speed,
        }
    }
}

impl<TV, TA, VR, V, A, J, T, SV> TrajectoryGeneratorBuilder<TV, TA, (), VR, V, A, J, T, SV> {
    pub fn angular_jerk_ref(
        self,
        angular_jerk_ref: AngularJerk,
    ) -> TrajectoryGeneratorBuilder<TV, TA, AngularJerk, VR, V, A, J, T, SV> {
        TrajectoryGeneratorBuilder {
            angular_speed_ref: self.angular_speed_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref,
            slalom_speed_ref: self.slalom_speed_ref,
            max_speed: self.max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_speed: self.search_speed,
        }
    }
}

impl<TV, TA, TJ, V, A, J, T, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, (), V, A, J, T, SV> {
    pub fn slalom_speed_ref(
        self,
        slalom_speed_ref: Speed,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, Speed, V, A, J, T, SV> {
        TrajectoryGeneratorBuilder {
            angular_speed_ref: self.angular_speed_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_speed_ref,
            max_speed: self.max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_speed: self.search_speed,
        }
    }
}

impl<TV, TA, TJ, VR, A, J, T, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, (), A, J, T, SV> {
    pub fn max_speed(
        self,
        max_speed: Speed,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, Speed, A, J, T, SV> {
        TrajectoryGeneratorBuilder {
            angular_speed_ref: self.angular_speed_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_speed_ref: self.slalom_speed_ref,
            max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_speed: self.search_speed,
        }
    }
}

impl<TV, TA, TJ, VR, V, J, T, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, (), J, T, SV> {
    pub fn max_acceleration(
        self,
        max_acceleration: Acceleration,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, Acceleration, J, T, SV> {
        TrajectoryGeneratorBuilder {
            angular_speed_ref: self.angular_speed_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_speed_ref: self.slalom_speed_ref,
            max_speed: self.max_speed,
            max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_speed: self.search_speed,
        }
    }
}

impl<TV, TA, TJ, VR, V, A, T, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, (), T, SV> {
    pub fn max_jerk(
        self,
        max_jerk: Jerk,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, Jerk, T, SV> {
        TrajectoryGeneratorBuilder {
            angular_speed_ref: self.angular_speed_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_speed_ref: self.slalom_speed_ref,
            max_speed: self.max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk,
            period: self.period,
            search_speed: self.search_speed,
        }
    }
}

impl<TV, TA, TJ, VR, V, A, J, SV> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, J, (), SV> {
    pub fn period(
        self,
        period: Time,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, J, Time, SV> {
        TrajectoryGeneratorBuilder {
            angular_speed_ref: self.angular_speed_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_speed_ref: self.slalom_speed_ref,
            max_speed: self.max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period,
            search_speed: self.search_speed,
        }
    }
}

impl<TV, TA, TJ, VR, V, A, J, T> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, J, T, ()> {
    pub fn search_speed(
        self,
        search_speed: Speed,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, J, T, Speed> {
        TrajectoryGeneratorBuilder {
            angular_speed_ref: self.angular_speed_ref,
            angular_acceleration_ref: self.angular_acceleration_ref,
            angular_jerk_ref: self.angular_jerk_ref,
            slalom_speed_ref: self.slalom_speed_ref,
            max_speed: self.max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
            search_speed,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn build_generator() -> TrajectoryGenerator {
        TrajectoryGeneratorBuilder::new()
            .max_speed(Speed::from_meter_per_second(1.0))
            .max_acceleration(Acceleration::from_meter_per_second_squared(10.0))
            .max_jerk(Jerk::from_meter_per_second_cubed(100.0))
            .angular_speed_ref(AngularSpeed::from_degree_per_second(180.0))
            .angular_acceleration_ref(AngularAcceleration::from_degree_per_second_squared(1800.0))
            .angular_jerk_ref(AngularJerk::from_degree_per_second_cubed(18000.0))
            .slalom_speed_ref(Speed::from_meter_per_second(0.6))
            .period(Time::from_seconds(0.001))
            .search_speed(Speed::from_meter_per_second(0.6))
            .build()
    }

    #[test]
    fn test_build() {
        build_generator();
    }

    #[test]
    fn test_search_trajectory() {
        use agent::TrajectoryGenerator;
        use core::f32::consts::PI;

        const EPSILON: f32 = 0.001;

        let period = Time::from_seconds(0.001);
        let search_speed = Speed::from_meter_per_second(0.2);

        let generator = TrajectoryGeneratorBuilder::new()
            .period(period)
            .max_speed(Speed::from_meter_per_second(2.0))
            .max_acceleration(Acceleration::from_meter_per_second_squared(0.7))
            .max_jerk(Jerk::from_meter_per_second_cubed(1.0))
            .search_speed(search_speed)
            .slalom_speed_ref(Speed::from_meter_per_second(0.27178875))
            .angular_speed_ref(AngularSpeed::from_radian_per_second(3.0 * PI))
            .angular_acceleration_ref(AngularAcceleration::from_radian_per_second_squared(
                36.0 * PI,
            ))
            .angular_jerk_ref(AngularJerk::from_radian_per_second_cubed(1200.0 * PI))
            .build();

        let trajectory = generator.generate_search(
            Pose::new(
                Distance::from_meters(0.045),
                Distance::from_meters(0.09),
                Angle::from_degree(90.0),
            ),
            RelativeDirection::Front,
        );

        let last = trajectory.last().unwrap();
        assert_relative_eq!(last.x.x.as_meters(), 0.045, epsilon = EPSILON);
        assert_relative_eq!(last.y.x.as_meters(), 0.18, epsilon = EPSILON);
    }
}
