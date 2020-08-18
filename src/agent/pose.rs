use quantities::{Angle, Distance};

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Pose {
    pub x: Distance,
    pub y: Distance,
    pub theta: Angle,
}

impl Pose {
    pub fn new(x: Distance, y: Distance, theta: Angle) -> Self {
        Self { x, y, theta }
    }
}
