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

fn _convert<N>(node: &Node<N>, square_width_half: Length) -> Pose {
    use AbsoluteDirection::*;

    Pose {
        x: (node.x() + 1) as f32 * square_width_half,
        y: (node.y() + 1) as f32 * square_width_half,
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
    }
}

//TODO: Write test.
impl<N, K> ICommandConverter<(Node<N>, K)> for CommandConverter {
    type Output = (Pose, K);

    fn convert(&self, (node, kind): (Node<N>, K)) -> Self::Output {
        (_convert(&node, self.square_width_half), kind)
    }
}

impl<N, INode, K> ICommandConverter<(INode, K)> for CommandConverter
where
    INode: core::ops::Deref<Target = Node<N>>,
{
    type Output = (Pose, K);

    fn convert(&self, (node, kind): (INode, K)) -> Self::Output {
        (_convert(node.deref(), self.square_width_half), kind)
    }
}

//NOTE: This struct is intended to be used by SearchOperator
#[derive(Clone, PartialEq, Debug)]
pub struct CommandConverter2 {
    square_width_half: Length,
    front_offset: Length,
}

impl CommandConverter2 {
    pub fn new(square_width: Length, front_offset: Length) -> Self {
        Self {
            square_width_half: square_width / 2.0,
            front_offset,
        }
    }
}

impl CommandConverter2 {
    const DEFAULT_SQUARE_WIDTH: Length = Length {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.09,
    };
}

impl Default for CommandConverter2 {
    fn default() -> Self {
        Self::new(Self::DEFAULT_SQUARE_WIDTH, Default::default())
    }
}

//TODO: Write test.
impl<N, K> ICommandConverter<(Node<N>, K)> for CommandConverter2 {
    type Output = (Pose, K);

    fn convert(&self, (node, kind): (Node<N>, K)) -> Self::Output {
        use AbsoluteDirection::*;

        let (dx, dy, theta) = if (node.x() ^ node.y()) & 1 == 1 {
            match node.direction() {
                North => (Default::default(), self.front_offset, 90.0),
                East => (self.front_offset, Default::default(), 0.0),
                South => (Default::default(), -self.front_offset, -90.0),
                West => (-self.front_offset, Default::default(), 180.0),
                _ => unreachable!("This struct is intended to be used by SearchOperator"),
            }
        } else {
            (
                Default::default(),
                Default::default(),
                match node.direction() {
                    North => 90.0,
                    East => 0.0,
                    South => -90.0,
                    West => 180.0,
                    _ => unreachable!("This struct is intended to be used by SearchOperator"),
                },
            )
        };
        (
            Pose {
                x: (node.x() + 1) as f32 * self.square_width_half + dx,
                y: (node.y() + 1) as f32 * self.square_width_half + dy,
                theta: Angle::new::<degree>(theta),
            },
            kind,
        )
    }
}
