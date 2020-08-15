extern crate components;

use heapless::consts::*;

use components::{
    administrator,
    maze::{AbsoluteDirection, Maze, NodeId, SearchNodeId, WallDirection, WallPosition},
    pattern::Pattern,
    solver::Solver,
};

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
        let maze = Maze::<U4, _>::new(cost);
        for wall in walls {
            maze.check_wall(wall, true);
        }
        let path = administrator::Solver::compute_shortest_path(&solver, &maze);
        assert_eq!(path.unwrap(), expected.as_slice());
    }
}

#[test]
fn test_next_node_candidates_u4() {
    use generic_array::arr;
    use AbsoluteDirection::*;

    let new = |x, y, dir| NodeId::<U4>::new(x, y, dir).unwrap();
    let new_search = |x, y, dir| SearchNodeId::<U4>::new(x, y, dir).unwrap();
    let new_wall = |x, y, z| WallPosition::<U4>::new(x, y, z).unwrap();

    let start = new(0, 0, North);
    let goals = arr![NodeId<U4>; new(2,0,West), new(2,0,South)];

    let solver = Solver::<NodeId<U4>, SearchNodeId<U4>, U256, _>::new(start, goals);

    let right = WallDirection::Right;
    let up = WallDirection::Up;

    let test_data = vec![(
        new_search(0, 1, North),
        vec![(new_wall(0, 0, right), true), (new_wall(0, 0, up), false)],
        vec![
            new_search(1, 2, East),
            new_search(0, 3, North),
            new_search(0, 1, South),
        ],
    )];

    for (current, walls, expected) in test_data {
        let maze = Maze::<U4, _>::new(cost);
        for (wall, exists) in walls {
            maze.check_wall(wall, exists);
        }
        let candidates = administrator::Solver::next_node_candidates(&solver, current, &maze);
        assert_eq!(candidates.unwrap(), expected.as_slice());
    }
}
