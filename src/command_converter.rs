use core::marker::PhantomData;

use uom::si::angle::degree;
use uom::si::f32::{Angle, Length};

use crate::data_types::{AbsoluteDirection, Pose};
use crate::node::Node;
use crate::operators::CommandConverter as ICommandConverter;

#[derive(Clone, PartialEq, Debug)]
pub struct CommandConverter {
    square_width_half: Length,
}

impl CommandConverter {
    pub fn new(square_width: Length) -> Self {
        Self {
            square_width_half: square_width / 2.0,
        }
    }
}

impl CommandConverter {
    const DEFAULT_SQUARE_WIDTH: Length = Length {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.09,
    };
}

impl Default for CommandConverter {
    fn default() -> Self {
        Self::new(Self::DEFAULT_SQUARE_WIDTH)
    }
}

//TODO: Write test.
impl<N, K> ICommandConverter<(Node<N>, K)> for CommandConverter {
    type Output = (Pose, K);

    fn convert(&self, (node, kind): (Node<N>, K)) -> Self::Output {
        use AbsoluteDirection::*;

        (
            Pose {
                x: (node.x() + 1) as f32 * self.square_width_half,
                y: (node.y() + 1) as f32 * self.square_width_half,
                theta: Angle::new::<degree>(match node.direction() {
                    East => 0.0,
                    NorthEast => 45.0,
                    North => 90.0,
                    NorthWest => 135.0,
                    West => 180.0,
                    SouthWest => -135.0,
                    South => -90.0,
                    SouthEast => -45.0,
                }),
            },
            kind,
        )
    }
}
