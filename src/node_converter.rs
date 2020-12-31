use core::convert::Infallible;

use uom::si::angle::degree;
use uom::si::f32::{Angle, Length};

use crate::commander::NodeConverter as INodeConverter;
use crate::data_types::{AbsoluteDirection, Pose};
use crate::node::{RunNode, SearchNode};

#[derive(Clone, PartialEq, Debug)]
pub struct NodeConverter {
    square_width_half: Length,
}

impl NodeConverter {
    pub fn new(square_width: Length) -> Self {
        Self {
            square_width_half: square_width / 2.0,
        }
    }
}

//TODO: Write test.
impl<N: core::fmt::Debug> INodeConverter<SearchNode<N>> for NodeConverter {
    type Error = Infallible;
    type Target = Pose;

    fn convert(&self, node: &SearchNode<N>) -> Result<Self::Target, Self::Error> {
        use AbsoluteDirection::*;

        Ok(Pose {
            x: (node.x() + 1) as f32 * self.square_width_half,
            y: (node.y() + 1) as f32 * self.square_width_half,
            theta: Angle::new::<degree>(match node.direction() {
                East => 0.0,
                North => 90.0,
                West => 180.0,
                South => -90.0,
                _ => unreachable!("Invariants of SearchNode broke: {:?}", node),
            }),
        })
    }
}

//TODO: Write test.
impl<N> INodeConverter<RunNode<N>> for NodeConverter {
    type Error = Infallible;
    type Target = Pose;

    fn convert(&self, node: &RunNode<N>) -> Result<Self::Target, Self::Error> {
        use AbsoluteDirection::*;

        Ok(Pose {
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
        })
    }
}
