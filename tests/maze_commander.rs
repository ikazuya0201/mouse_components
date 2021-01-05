extern crate components;

use typenum::consts::*;

use components::{
    data_types::{AbsoluteDirection, Pattern, RunNode, SearchNode, Wall},
    impls::{Commander, Maze, NodeConverter, PoseConverter, WallConverter, WallManager},
    prelude::*,
    traits::Math,
    utils::probability::Probability,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MathFake;

impl Math for MathFake {
    fn sqrtf(val: f32) -> f32 {
        val.sqrt()
    }

    fn sinf(val: f32) -> f32 {
        val.sin()
    }

    fn cosf(val: f32) -> f32 {
        val.cos()
    }

    fn expf(val: f32) -> f32 {
        val.exp()
    }

    fn atan2f(x: f32, y: f32) -> f32 {
        x.atan2(y)
    }

    fn rem_euclidf(lhs: f32, rhs: f32) -> f32 {
        lhs.rem_euclid(rhs)
    }
}

fn cost(pattern: Pattern) -> u16 {
    use Pattern::*;

    match pattern {
        Straight(x) => 10 * x,
        StraightDiagonal(x) => 7 * x,
        Search90 => 8,
        FastRun45 => 12,
        FastRun90 => 15,
        FastRun135 => 20,
        FastRun180 => 25,
        FastRunDiagonal90 => 15,
        SpinBack => 15,
    }
}

#[test]
fn test_compute_shortest_path_u4() {
    use core::ops::Mul;
    use AbsoluteDirection::*;

    type Size = U4;
    type MaxPathLength = <<Size as Mul<Size>>::Output as Mul<U16>>::Output;

    let new = |x, y, dir| RunNode::<Size>::new(x, y, dir, cost).unwrap();
    let new_wall = |x, y, z| Wall::<Size>::new(x, y, z).unwrap();

    let start = new(0, 0, North);
    let goals = vec![new(2, 0, West), new(2, 0, South)];
    let start_search_node = SearchNode::<Size>::new(0, 1, North, cost).unwrap();

    let test_data = vec![
        (vec![], vec![new(0, 0, North), new(2, 0, South)]),
        (
            vec![
                new_wall(0, 0, false),
                new_wall(0, 1, false),
                new_wall(0, 2, true),
                new_wall(1, 0, true),
                new_wall(1, 1, true),
                new_wall(1, 2, false),
                new_wall(2, 1, false),
                new_wall(2, 2, true),
                new_wall(3, 1, true),
            ],
            vec![
                new(0, 0, North),
                new(0, 2, North),
                new(1, 4, NorthEast),
                new(2, 5, NorthEast),
                new(4, 6, East),
                new(5, 4, SouthWest),
                new(4, 2, South),
                new(2, 0, West),
            ],
        ),
    ];

    for (walls, expected) in test_data {
        let wall_manager = WallManager::new(Probability::new(0.1).unwrap());
        for wall in walls {
            wall_manager.update(&wall, &Probability::one());
        }

        let pose_converter = PoseConverter::<Size, MathFake>::default();
        let wall_converter = WallConverter::new(cost);
        let maze = Maze::<_, _, _, MathFake>::new(wall_manager, pose_converter, wall_converter);
        let node_converter = NodeConverter::default();
        let commander = Commander::<_, _, _, MaxPathLength, _, _>::new(
            start,
            goals.clone(),
            start_search_node.clone(),
            maze,
            node_converter,
        );

        let path = commander.compute_shortest_path();
        assert_eq!(path.unwrap(), expected.as_slice());
    }
}
