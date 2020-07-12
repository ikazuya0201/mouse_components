#[macro_use]
mod macros;

pub mod slalom_generator;
pub mod straight_generator;

mod trajectory;

use quantities::{
    Acceleration, AngularAcceleration, AngularJerk, AngularSpeed, Distance, Jerk, Speed, Time,
};

use slalom_generator::SlalomGenerator;
use straight_generator::StraightGenerator;

pub struct TrajectoryGenerator {
    straight_generator: StraightGenerator<Distance>,
    slalom_generator: SlalomGenerator,
}

pub struct TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, J, T> {
    max_angular_speed: TV,
    max_angular_acceleration: TA,
    max_angular_jerk: TJ,
    slalom_vref: VR,
    max_speed: V,
    max_acceleration: A,
    max_jerk: J,
    period: T,
}

impl TrajectoryGeneratorBuilder<(), (), (), (), (), (), (), ()> {
    pub fn new() -> Self {
        Self {
            max_angular_speed: (),
            max_angular_acceleration: (),
            max_angular_jerk: (),
            slalom_vref: (),
            max_speed: (),
            max_acceleration: (),
            max_jerk: (),
            period: (),
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
    >
{
    pub fn build(self) -> TrajectoryGenerator {
        TrajectoryGenerator {
            straight_generator: StraightGenerator::new(
                self.max_speed,
                self.max_acceleration,
                self.max_jerk,
            ),
            slalom_generator: SlalomGenerator::new(
                self.max_angular_speed,
                self.max_angular_acceleration,
                self.max_angular_jerk,
                self.slalom_vref,
                self.period,
            ),
        }
    }
}

impl<TA, TJ, VR, V, A, J, T> TrajectoryGeneratorBuilder<(), TA, TJ, VR, V, A, J, T> {
    pub fn max_angular_speed(
        self,
        max_angular_speed: AngularSpeed,
    ) -> TrajectoryGeneratorBuilder<AngularSpeed, TA, TJ, VR, V, A, J, T> {
        TrajectoryGeneratorBuilder {
            max_angular_speed,
            max_angular_acceleration: self.max_angular_acceleration,
            max_angular_jerk: self.max_angular_jerk,
            slalom_vref: self.slalom_vref,
            max_speed: self.max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
        }
    }
}

impl<TV, TJ, VR, V, A, J, T> TrajectoryGeneratorBuilder<TV, (), TJ, VR, V, A, J, T> {
    pub fn max_angular_acceleration(
        self,
        max_angular_acceleration: AngularAcceleration,
    ) -> TrajectoryGeneratorBuilder<TV, AngularAcceleration, TJ, VR, V, A, J, T> {
        TrajectoryGeneratorBuilder {
            max_angular_speed: self.max_angular_speed,
            max_angular_acceleration,
            max_angular_jerk: self.max_angular_jerk,
            slalom_vref: self.slalom_vref,
            max_speed: self.max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
        }
    }
}

impl<TV, TA, VR, V, A, J, T> TrajectoryGeneratorBuilder<TV, TA, (), VR, V, A, J, T> {
    pub fn max_angular_jerk(
        self,
        max_angular_jerk: AngularJerk,
    ) -> TrajectoryGeneratorBuilder<TV, TA, AngularJerk, VR, V, A, J, T> {
        TrajectoryGeneratorBuilder {
            max_angular_speed: self.max_angular_speed,
            max_angular_acceleration: self.max_angular_acceleration,
            max_angular_jerk,
            slalom_vref: self.slalom_vref,
            max_speed: self.max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
        }
    }
}

impl<TV, TA, TJ, V, A, J, T> TrajectoryGeneratorBuilder<TV, TA, TJ, (), V, A, J, T> {
    pub fn slalom_vref(
        self,
        slalom_vref: Speed,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, Speed, V, A, J, T> {
        TrajectoryGeneratorBuilder {
            max_angular_speed: self.max_angular_speed,
            max_angular_acceleration: self.max_angular_acceleration,
            max_angular_jerk: self.max_angular_jerk,
            slalom_vref,
            max_speed: self.max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
        }
    }
}

impl<TV, TA, TJ, VR, A, J, T> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, (), A, J, T> {
    pub fn max_speed(
        self,
        max_speed: Speed,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, Speed, A, J, T> {
        TrajectoryGeneratorBuilder {
            max_angular_speed: self.max_angular_speed,
            max_angular_acceleration: self.max_angular_acceleration,
            max_angular_jerk: self.max_angular_jerk,
            slalom_vref: self.slalom_vref,
            max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
        }
    }
}

impl<TV, TA, TJ, VR, V, J, T> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, (), J, T> {
    pub fn max_acceleration(
        self,
        max_acceleration: Acceleration,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, Acceleration, J, T> {
        TrajectoryGeneratorBuilder {
            max_angular_speed: self.max_angular_speed,
            max_angular_acceleration: self.max_angular_acceleration,
            max_angular_jerk: self.max_angular_jerk,
            slalom_vref: self.slalom_vref,
            max_speed: self.max_speed,
            max_acceleration,
            max_jerk: self.max_jerk,
            period: self.period,
        }
    }
}

impl<TV, TA, TJ, VR, V, A, T> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, (), T> {
    pub fn max_jerk(
        self,
        max_jerk: Jerk,
    ) -> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, Jerk, T> {
        TrajectoryGeneratorBuilder {
            max_angular_speed: self.max_angular_speed,
            max_angular_acceleration: self.max_angular_acceleration,
            max_angular_jerk: self.max_angular_jerk,
            slalom_vref: self.slalom_vref,
            max_speed: self.max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk,
            period: self.period,
        }
    }
}

impl<TV, TA, TJ, VR, V, A, J> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, J, ()> {
    pub fn period(self, period: Time) -> TrajectoryGeneratorBuilder<TV, TA, TJ, VR, V, A, J, Time> {
        TrajectoryGeneratorBuilder {
            max_angular_speed: self.max_angular_speed,
            max_angular_acceleration: self.max_angular_acceleration,
            max_angular_jerk: self.max_angular_jerk,
            slalom_vref: self.slalom_vref,
            max_speed: self.max_speed,
            max_acceleration: self.max_acceleration,
            max_jerk: self.max_jerk,
            period,
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
            .max_angular_speed(AngularSpeed::from_degree_per_second(180.0))
            .max_angular_acceleration(AngularAcceleration::from_degree_per_second_squared(1800.0))
            .max_angular_jerk(AngularJerk::from_degree_per_second_cubed(18000.0))
            .slalom_vref(Speed::from_meter_per_second(0.6))
            .period(Time::from_seconds(0.001))
            .build()
    }

    #[test]
    fn test_build() {
        build_generator();
    }
}
