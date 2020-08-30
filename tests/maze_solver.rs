extern crate components;

use heapless::consts::*;

use components::{
    data_types::{AbsoluteDirection, NodeId, Pattern, SearchNodeId, WallDirection, WallPosition},
    impls::{MazeBuilder, Solver},
    prelude::*,
    traits::Math,
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
    use generic_array::arr;
    use AbsoluteDirection::*;

    let new = |x, y, dir| NodeId::<U4>::new(x, y, dir).unwrap();
    let new_wall = |x, y, z| WallPosition::<U4>::new(x, y, z).unwrap();

    let start = new(0, 0, North);
    let goals = arr![NodeId<U4>; new(2,0,West), new(2,0,South)];

    let solver = Solver::<NodeId<U4>, SearchNodeId<U4>, U256, _>::new(start, goals);

    let right = WallDirection::Right;
    let up = WallDirection::Up;

    let test_data = vec![
        (vec![], vec![new(0, 0, North), new(2, 0, South)]),
        (
            vec![
                new_wall(0, 0, right),
                new_wall(0, 1, right),
                new_wall(0, 2, up),
                new_wall(1, 0, up),
                new_wall(1, 1, up),
                new_wall(1, 2, right),
                new_wall(2, 1, right),
                new_wall(2, 2, up),
                new_wall(3, 1, up),
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
        let maze = MazeBuilder::new().costs(cost).build::<U4, MathFake>();
        for wall in walls {
            maze.check_wall(wall, true);
        }
        let path = solver.compute_shortest_path(&maze);
        assert_eq!(path.unwrap(), expected.as_slice());
    }
}

macro_rules! next_node_candidates_tests {
    ($($name: ident: $value: expr,)*) => {
        $(
                #[test]
                fn $name() {
                    use generic_array::arr;
                    use AbsoluteDirection::*;
                    use WallDirection::*;

                    let new = |x, y, dir| NodeId::<U4>::new(x, y, dir).unwrap();
                    let new_search = |x, y, dir| SearchNodeId::<U4>::new(x, y, dir).unwrap();
                    let new_wall = |x, y, z| WallPosition::<U4>::new(x, y, z).unwrap();

                    let start = new(0, 0, North);
                    let goals = arr![NodeId<U4>; new(2,0,West), new(2,0,South)];

                    let solver = Solver::<NodeId<U4>, SearchNodeId<U4>, U256, _>::new(start, goals);

                    let (input, expected) = $value;
                    let current = input.0;
                    let walls = input.1;

                    let current = new_search(current.0, current.1, current.2);
                    let maze = MazeBuilder::new().costs(cost).build::<U4, MathFake>();
                    for (wall, exists) in walls.into_iter().map(|wall| (new_wall(wall.0,wall.1,wall.2), wall.3)) {
                        maze.check_wall(wall, exists);
                    }
                    let expected = expected.into_iter().map(|input| new_search(input.0, input.1, input.2)).collect::<Vec<_>>();
                    let candidates = solver.next_node_candidates(current, &maze);
                    assert_eq!(candidates.unwrap(), expected.as_slice());
                }
        )*
    }
}

next_node_candidates_tests! {
    test_next_node_candidates1: (
        (
            (0,1,North),
            vec![
                (0,0,Right,true),
                (0,0,Up,false),
            ],
        ),
        vec![
            (1,2,East),
            (0,3,North),
            (0,1,South),
        ],
    ),
    test_next_node_candidates2: (
        (
            (1,2,East),
            vec![
                (0,0,Right,true),
                (0,0,Up,false),
                (0,1,Right,false),
                (0,1,Up,false),
            ],
        ),
        vec![
            (3,2,East),
            (2,3,North),
            (2,1,South),
            (1,2,West),
        ],
    ),
}
