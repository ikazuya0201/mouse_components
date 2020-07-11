#[macro_use]
mod macros;

mod straight_generator;
mod trajectory;

use quantities::Distance;

use straight_generator::StraightGenerator;

pub struct TrajectoryGenerator {
    straight_generator: StraightGenerator<Distance>,
}

impl TrajectoryGenerator {}
