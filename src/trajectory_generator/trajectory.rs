use quantities::{Acceleration, Distance, Jerk, Speed};

#[derive(Clone, Debug)]
pub struct Target {
    pub x: Distance,
    pub v: Speed,
    pub a: Acceleration,
    pub j: Jerk,
}
