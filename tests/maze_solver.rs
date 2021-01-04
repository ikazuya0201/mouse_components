extern crate components;

use core::cell::RefCell;
use core::convert::TryInto;
use core::ops::Mul;
use std::rc::Rc;

use typenum::{consts::*, PowerOfTwo, Unsigned};
use uom::si::{
    angle::degree,
    f32::{Angle, Length},
    length::meter,
};

use components::{
    data_types::{
        AbsoluteDirection, NodeId, Obstacle, Pattern, Pose, Position, RelativeDirection,
        SearchKind, SearchNodeId, WallDirection, WallPosition,
    },
    impls::{Maze, MazeBuilder, SearchOperator, Solver},
    prelude::*,
    traits::{Math, SearchAgent},
    utils::{array_length::ArrayLength, sample::Sample},
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
    let start_search_node = SearchNodeId::new(0, 1, North).unwrap();

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
        let solver = Solver::<NodeId<U4>, SearchNodeId<U4>, U256, _, _>::new(
            start,
            goals,
            start_search_node,
            Rc::new(maze),
        );

        let path = solver.compute_shortest_path();
        assert_eq!(path.unwrap(), expected.as_slice());
    }
}

struct AgentMock<N, M>
where
    N: Mul<N> + Unsigned + PowerOfTwo,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
{
    maze: Rc<Maze<N, M>>,
    current: RefCell<SearchNodeId<N>>,
    square_width: Length,
    wall_width: Length,
}

impl<N, M> AgentMock<N, M>
where
    N: Mul<N> + Unsigned + PowerOfTwo,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
{
    fn new(
        maze: Rc<Maze<N, M>>,
        current: SearchNodeId<N>,
        square_width: Length,
        wall_width: Length,
    ) -> Self {
        Self {
            maze,
            current: RefCell::new(current),
            square_width,
            wall_width,
        }
    }

    fn position_to_obstacle(&self, position: Position<N>) -> Obstacle {
        let convert = |x: i16| (x + 1) as f32 * self.square_width / 2.0;
        let pose = if position.x() % 2 == 0 {
            Pose {
                x: convert(position.x()),
                y: convert(position.y() - 1),
                theta: Angle::new::<degree>(90.0),
            }
        } else {
            Pose {
                x: convert(position.x() - 1),
                y: convert(position.y()),
                theta: Angle::new::<degree>(0.0),
            }
        };
        let distance = if self.maze.is_wall_by_position(position) {
            self.square_width / 2.0 - self.wall_width / 2.0
        } else {
            self.square_width * 3.0 / 2.0 - self.wall_width / 2.0
        };
        Obstacle {
            source: pose,
            distance: Sample {
                mean: distance,
                standard_deviation: Length::new::<meter>(0.001),
            },
        }
    }

    fn pose_to_node(&self, pose: Pose) -> SearchNodeId<N> {
        let length_to_i16 = |length: Length| {
            let ld = length - self.square_width / 2.0;
            ((ld * 2.0 / self.square_width).value + 0.01) as i16
        };
        let x = length_to_i16(pose.x);
        let y = length_to_i16(pose.y);

        let dir = if pose.theta < Angle::new::<degree>(45.0) {
            AbsoluteDirection::East
        } else if pose.theta < Angle::new::<degree>(135.0) {
            AbsoluteDirection::North
        } else if pose.theta < Angle::new::<degree>(225.0) {
            AbsoluteDirection::West
        } else {
            AbsoluteDirection::South
        };
        assert!(x >= 0);
        assert!(y >= 0);
        SearchNodeId::new(x as u16, y as u16, dir)
            .unwrap_or_else(|err| unreachable!("node out of bound: {:?}", err))
    }
}

impl<N, M> SearchAgent<(Pose, SearchKind)> for AgentMock<N, M>
where
    N: Mul<N> + Unsigned + PowerOfTwo + core::fmt::Debug,
    <N as Mul<N>>::Output: Mul<U2>,
    <<N as Mul<N>>::Output as Mul<U2>>::Output: ArrayLength<f32>,
{
    type Error = ();
    type Obstacle = Obstacle;
    type Obstacles = Vec<Self::Obstacle>;

    fn update_state(&self) {}

    fn get_obstacles(&self) -> Self::Obstacles {
        let current = self.current.borrow().to_node();
        let relative = |x: i16, y: i16| {
            current
                .get_relative_position(x, y, AbsoluteDirection::North)
                .unwrap()
        };
        let positions = vec![relative(1, 1), relative(0, 2), relative(-1, 1)];
        let obstacles = positions
            .into_iter()
            .map(|position| self.position_to_obstacle(position))
            .collect();
        obstacles
    }

    fn set_command(&self, command: &(Pose, SearchKind)) {
        use RelativeDirection::*;

        let current = self.pose_to_node(command.0).to_node();

        let relative = |x: i16, y: i16, dir: &RelativeDirection| {
            current
                .get_relative_node(x, y, *dir, AbsoluteDirection::North)
                .expect("failed to convert to relative node")
                .try_into()
                .unwrap_or_else(|err| {
                    unreachable!(
                        "failed to convert to search node id: current: {:?}, relative: {:?}",
                        current, err,
                    )
                })
        };
        let next = match command.1 {
            SearchKind::Init => self
                .pose_to_node(command.0)
                .to_node()
                .try_into()
                .expect("cannot convert to search node id"),
            SearchKind::Search(direction) => match direction {
                Right => relative(1, 1, &direction),
                Left => relative(-1, 1, &direction),
                Front => relative(0, 2, &direction),
                Back => relative(0, 0, &direction),
                _ => unreachable!(),
            },
        };
        self.current.replace(next);
    }

    fn track_next(&self) -> Result<(), Self::Error> {
        Ok(())
    }
}

macro_rules! search_tests {
    ($($name: ident: ($size: ty, $goal: ty, $value: expr),)*) => {
        $(
            #[ignore]
            #[test]
            fn $name() -> std::io::Result<()> {
                use generic_array::GenericArray;
                use std::fs;
                use AbsoluteDirection::*;

                let (goals, filename) = $value;
                let root = env!("CARGO_MANIFEST_DIR").to_string();
                let file_name = root + "/mazes/" + filename;
                let start = NodeId::new(0, 0, North).unwrap();
                let goals = goals.into_iter()
                    .map(|goal| NodeId::new(goal.0, goal.1, goal.2).unwrap())
                    .collect::<GenericArray<NodeId<$size>, $goal>>();

                let square_width = Length::new::<meter>(0.09);
                let wall_width = Length::new::<meter>(0.006);
                let ignore_radius = Length::new::<meter>(0.01);

                let contents = fs::read_to_string(file_name)?;
                let agent_maze = Rc::new(
                    MazeBuilder::new()
                        .costs(cost)
                        .init_with(&contents)
                        .square_width(square_width)
                        .wall_width(wall_width)
                        .ignore_radius_from_pillar(ignore_radius)
                        .build::<$size, MathFake>(),
                );

                let start_search_node = SearchNodeId::new(0, 1, North).unwrap();
                let agent = Rc::new(AgentMock::new(
                    Rc::clone(&agent_maze),
                    start_search_node,
                    square_width,
                    wall_width,
                ));

                let agent_solver = Solver::<NodeId<$size>, SearchNodeId<$size>, PathLen, _,_>::new(
                    start,
                    goals,
                    start_search_node,
                    Rc::clone(&agent_maze),
                );

                let maze = Rc::new(
                    MazeBuilder::new()
                        .costs(cost)
                        .square_width(square_width)
                        .wall_width(wall_width)
                        .ignore_radius_from_pillar(ignore_radius)
                        .build::<$size, MathFake>(),
                );

                use std::ops::Mul;
                type PathLen = <<$size as Mul<$size>>::Output as Mul<U16>>::Output;
                let solver = Rc::new(Solver::<NodeId<$size>, SearchNodeId<$size>, PathLen, _,_>::new(
                    start, goals, start_search_node, Rc::clone(&maze),
                ));

                let start_pose = Pose {
                    x: Length::new::<meter>(0.045),
                    y: Length::new::<meter>(0.045),
                    theta: Angle::new::<degree>(90.0),
                };
                let operator = SearchOperator::<
                    (),
                    Obstacle,
                    _,
                    _,
                >::new(
                    (start_pose, SearchKind::Init),
                    (),
                    Rc::clone(&agent),
                    Rc::clone(&solver),
                );

                while operator.run().is_err() {
                    operator.tick().expect("Should never panic.");
                }
                assert_eq!(
                    solver.compute_shortest_path(),
                    agent_solver.compute_shortest_path()
                );
                Ok(())
            }
        )*
    }
}

search_tests! {
    test_search1: (U4, U2, (vec![(2,0,South), (2,0,West)], "maze1.dat")),
    test_search2: (U16, U8, (vec![
        (15, 14, NorthEast),
        (15, 14, NorthWest),
        (14, 15, NorthEast),
        (14, 15, SouthEast),
        (16, 15, NorthWest),
        (16, 15, SouthWest),
        (15, 16, SouthEast),
        (15, 16, SouthWest),
    ],
    "maze2.dat")),
}
