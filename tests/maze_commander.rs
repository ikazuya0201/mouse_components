extern crate components;

use std::rc::Rc;

use components::{
    commanders::SearchCommander,
    mazes::Maze,
    nodes::{Node, RunNode, SearchNode},
    pattern_converters::LinearPatternConverter,
    types::data::{AbsoluteDirection, SearchKind},
    utils::probability::Probability,
    wall_manager::WallManager,
};
use AbsoluteDirection::*;

fn test_compute_shortest_path<const N: usize>(
    maze_str: &str,
    start: RunNode<N>,
    goals: &[RunNode<N>],
    expected: &[RunNode<N>],
) {
    let wall_manager = WallManager::with_str(Probability::new(0.1).unwrap(), maze_str);
    let maze =
        Maze::<_, _, SearchNode<N>>::new(Rc::new(wall_manager), LinearPatternConverter::default());
    let current: Node<N> = start.clone().into();
    let commander = SearchCommander::<_, _, SearchNode<N>, _, _>::new(
        start,
        &goals,
        current,
        SearchKind::Init,
        SearchKind::Final,
        maze,
    );

    let path = commander.compute_shortest_path();
    assert_eq!(path.unwrap(), expected);
}

fn new<const N: usize>((x, y, dir): (i8, i8, AbsoluteDirection)) -> RunNode<N> {
    RunNode::<N>::new(x, y, dir).unwrap()
}

#[test]
fn test_compute_shortest_path1() {
    test_compute_shortest_path::<4>(
        r"+---+---+---+---+
|               |
+   +   +   +   +
|               |
+   +   +   +   +
|               |
+   +   +   +   +
|               |
+---+---+---+---+",
        new((0, 0, North)),
        &vec![(2, 0, West), (2, 0, South)]
            .into_iter()
            .map(new)
            .collect::<Vec<_>>(),
        &vec![(0, 0, North), (2, 0, South)]
            .into_iter()
            .map(new)
            .collect::<Vec<_>>(),
    )
}

#[test]
fn test_compute_shortest_path2() {
    test_compute_shortest_path::<4>(
        r"+---+---+---+---+
|               |
+---+   +---+   +
|       |       |
+   +---+   +---+
|   |       |   |
+   +---+   +   +
|   |           |
+---+---+---+---+",
        new((0, 0, North)),
        &vec![(2, 0, West), (2, 0, South)]
            .into_iter()
            .map(new)
            .collect::<Vec<_>>(),
        &vec![
            (0, 0, North),
            (0, 2, North),
            (1, 4, NorthEast),
            (2, 5, NorthEast),
            (4, 6, East),
            (5, 4, SouthWest),
            (4, 2, South),
            (2, 0, West),
        ]
        .into_iter()
        .map(new)
        .collect::<Vec<_>>(),
    )
}

#[test]
fn test_compute_shortest_path3() {
    test_compute_shortest_path::<16>(
        include_str!("../mazes/maze2.dat"),
        new((0, 24, South)),
        &vec![(0, 0, South)].into_iter().map(new).collect::<Vec<_>>(),
        &vec![
            (0, 24, South),
            (0, 20, South),
            (1, 18, SouthEast),
            (2, 16, South),
            (2, 14, South),
            (1, 12, SouthWest),
            (0, 10, South),
            (0, 0, South),
        ]
        .into_iter()
        .map(new)
        .collect::<Vec<_>>(),
    )
}
