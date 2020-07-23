mod slalom_generator;
mod spin_generator;
mod straight_generator;
mod trajectory;

use auto_enums::auto_enum;
use quantities::{
    Acceleration, Angle, AngularAcceleration, AngularJerk, AngularSpeed, Distance, Jerk, Speed,
    Time,
};

use super::agent;
use crate::agent::pose::Pose;
use crate::maze::RelativeDirection;
use slalom_generator::SlalomGenerator;
use spin_generator::SpinGenerator;
use straight_generator::StraightTrajectoryGenerator;
pub use trajectory::{Target, Trajectory};

pub struct TrajectoryGenerator {
    straight_generator: StraightTrajectoryGenerator,
    slalom_generator: SlalomGenerator,
    spin_generator: SpinGenerator,
    search_speed: Speed,
}

impl agent::TrajectoryGenerator<Pose, Trajectory, RelativeDirection> for TrajectoryGenerator {
    type Trajectory = impl Iterator<Item = Trajectory>;

    fn generate_search(&self, pose: Pose, direction: RelativeDirection) -> Self::Trajectory {
        self.generate_search_trajectory(pose, direction)
    }
}

impl TrajectoryGenerator {
    #[auto_enum]
    pub fn generate_search_trajectory(
        &self,
        pose: Pose,
        direction: RelativeDirection,
    ) -> impl Iterator<Item = Trajectory> {
        use RelativeDirection::*;

        #[auto_enum(Iterator)]
        match direction {
            Right => self
                .slalom_generator
                .generate(
                    pose.x,
                    pose.y,
                    pose.theta,
                    Angle::from_degree(90.0),
                    self.search_speed,
                )
                .map(|target| Trajectory::Move(target)),
            Left => self
                .slalom_generator
                .generate(
                    pose.x,
                    pose.y,
                    pose.theta,
                    Angle::from_degree(-90.0),
                    self.search_speed,
                )
                .map(|target| Trajectory::Move(target)),
            Front => {
                let distance = Distance::from_meters(0.09); //TODO: use configurable value
                let x_end = pose.x + distance * pose.theta.cos();
                let y_end = pose.y + distance * pose.theta.sin();
                self.straight_generator
                    .generate(
                        pose.x,
                        pose.y,
                        x_end,
                        y_end,
                        self.search_speed,
                        self.search_speed,
                    )
                    .map(|target| Trajectory::Move(target))
            }
            Back => {
                let distance = Distance::from_meters(0.045); //TODO: use configurable value
                let x_end = pose.x + distance * pose.theta.cos();
                let y_end = pose.y + distance * pose.theta.sin();
                self.straight_generator
                    .generate(
                        pose.x,
                        pose.y,
                        x_end,
                        y_end,
                        self.search_speed,
                        Default::default(),
                    )
                    .map(|target| Trajectory::Move(target))
                    .chain(
                        self.spin_generator
                            .generate(pose.theta, Angle::from_degree(180.0))
                            .map(|target| Trajectory::Spin(target)),
                    )
                    .chain(
                        self.straight_generator
                            .generate(
                                x_end,
                                y_end,
                                pose.x,
                                pose.y,
                                Default::default(),
                                self.search_speed,
                            )
                            .map(|target| Trajectory::Move(target)),
                    )
            }
            _ => unreachable!(),
        }
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
        TrajectoryGenerator {
            straight_generator: StraightTrajectoryGenerator::new(
                self.max_speed,
                self.max_acceleration,
                self.max_jerk,
                self.period,
            ),
            slalom_generator: SlalomGenerator::new(
                self.angular_speed_ref,
                self.angular_acceleration_ref,
                self.angular_jerk_ref,
                self.slalom_speed_ref,
                self.period,
            ),
            spin_generator: SpinGenerator::new(
                self.angular_speed_ref,
                self.angular_acceleration_ref,
                self.angular_jerk_ref,
                self.period,
            ),
            search_speed: self.search_speed,
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
}
