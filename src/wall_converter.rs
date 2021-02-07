use heapless::Vec;
use typenum::{consts::*, Unsigned};

use crate::mazes::WallConverter as IWallConverter;
use crate::node::{NodeCreationError, Pattern, SearchNode};
use crate::types::data::AbsoluteDirection;
use crate::utils::forced_vec::ForcedVec;
use crate::wall_manager::Wall;

pub struct WallConverter {
    cost: fn(Pattern) -> u16,
}

impl WallConverter {
    pub fn new(cost: fn(Pattern) -> u16) -> Self {
        Self { cost }
    }
}

impl<N> IWallConverter<Wall<N>> for WallConverter
where
    N: Unsigned,
{
    type Error = NodeCreationError;
    type SearchNode = SearchNode<N>;
    type SearchNodes = Vec<Self::SearchNode, U2>;

    fn convert(&self, wall: &Wall<N>) -> Result<Self::SearchNodes, Self::Error> {
        use AbsoluteDirection::*;

        let mut nodes = ForcedVec::new();
        let (dx, dy, dirs) = if wall.is_top() {
            (0, 1, [North, South])
        } else {
            (1, 0, [East, West])
        };
        let x = (wall.x() * 2 + dx) as i16;
        let y = (wall.y() * 2 + dy) as i16;
        nodes.push(SearchNode::<N>::new(x, y, dirs[0], self.cost)?);
        nodes.push(SearchNode::<N>::new(x, y, dirs[1], self.cost)?);
        Ok(nodes.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_convert_wall() {
        use AbsoluteDirection::*;

        fn cost(_pattern: Pattern) -> u16 {
            unreachable!()
        }

        let test_cases = vec![
            ((0, 0, true), (vec![(0, 1, North), (0, 1, South)])),
            ((0, 0, false), (vec![(1, 0, East), (1, 0, West)])),
            ((1, 0, true), (vec![(2, 1, North), (2, 1, South)])),
            ((1, 1, false), (vec![(3, 2, East), (3, 2, West)])),
        ];

        type Size = U4;
        let converter = WallConverter::new(cost);

        use std::vec::Vec;

        for (wall, expected) in test_cases {
            let wall = Wall::<Size>::new(wall.0, wall.1, wall.2).unwrap();
            let expected = expected
                .into_iter()
                .map(|node| SearchNode::<Size>::new(node.0, node.1, node.2, cost).unwrap())
                .collect::<Vec<_>>();
            let nodes = converter
                .convert(&wall)
                .unwrap()
                .into_iter()
                .collect::<Vec<_>>();

            assert_eq!(nodes, expected);
        }
    }
}
