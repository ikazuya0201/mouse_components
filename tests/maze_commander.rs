extern crate components;

use components::{
    commanders::SearchCommander,
    mazes::Maze,
    nodes::{Node, RunNode, SearchNode},
    types::data::{AbsoluteDirection, Pattern, SearchKind, Wall},
    utils::probability::Probability,
    wall_manager::WallManager,
};
use typenum::consts::*;

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
    use AbsoluteDirection::*;

    type Size = U4;

    let new = |x, y, dir| RunNode::<Size>::new(x, y, dir).unwrap();
    let new_wall = |x, y, z| Wall::<Size>::new(x, y, z).unwrap();

    let start = new(0, 0, North);
    let goals = vec![new(2, 0, West), new(2, 0, South)];

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

        let maze = Maze::<_, _, _, SearchNode<Size>>::new(&wall_manager, cost);
        let current: Node<Size> = start.clone().into();
        let commander = SearchCommander::<_, _, SearchNode<Size>, _, _>::new(
            start,
            &goals,
            current,
            SearchKind::Init,
            SearchKind::Final,
            maze,
        );

        let path = commander.compute_shortest_path();
        assert_eq!(path.unwrap(), expected.as_slice());
    }
}
