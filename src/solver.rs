use alloc::rc::Rc;
use core::cell::{Cell, RefCell};
use core::cmp::Reverse;
use core::convert::TryInto;
use core::fmt::Debug;
use core::marker::PhantomData;

use generic_array::GenericArray;
use heap::BinaryHeap;
use heapless::{consts::*, Vec};
use num::{Bounded, Saturating};

use crate::data_types::{Pose, SearchKind};
use crate::operators::{FinishError, RunCommander, SearchCommander};
use crate::utils::{array_length::ArrayLength, itertools::repeat_n};

pub trait GraphConverter<Node> {
    type SearchNode;
    type SearchNodes: IntoIterator<Item = Self::SearchNode>;

    ///checker nodes are search nodes on which agent can check
    ///if walls on the given path exist.
    fn convert_to_checker_nodes<Nodes: IntoIterator<Item = Node>>(
        &self,
        path: Nodes,
    ) -> Self::SearchNodes;
}

pub trait Graph<Node> {
    type Cost;
    type Edges: IntoIterator<Item = (Node, Self::Cost)>;

    fn successors(&self, node: &Node) -> Self::Edges;
    fn predecessors(&self, node: &Node) -> Self::Edges;
}

pub trait ObstacleInterpreter<Obstacle> {
    fn interpret_obstacles<Obstacles: IntoIterator<Item = Obstacle>>(&self, obstacles: Obstacles);
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct CannotCheckError;

pub trait NodeChecker<Node> {
    fn is_available(&self, node: &Node) -> Result<bool, CannotCheckError>;
}

pub trait Converter<Source> {
    type Error;
    type Target;

    fn convert(&self, source: &Source) -> Result<Self::Target, Self::Error>;
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
enum State {
    Waiting,
    Solving,
}

type CandNum = U4;

pub struct Solver<Node, SearchNode, Max, GoalSize, Maze>
where
    GoalSize: ArrayLength<Node>,
{
    start: Node,
    goals: GenericArray<Node, GoalSize>,
    current: Cell<SearchNode>,
    state: Cell<State>,
    candidates: RefCell<Vec<SearchNode, CandNum>>,
    maze: Rc<Maze>,
    _node_max: PhantomData<fn() -> Max>,
}

impl<Node, SearchNode, Max, GoalSize, Maze> Solver<Node, SearchNode, Max, GoalSize, Maze>
where
    GoalSize: ArrayLength<Node>,
{
    pub fn new(
        start: Node,
        goals: GenericArray<Node, GoalSize>,
        search_start: SearchNode,
        maze: Rc<Maze>,
    ) -> Self {
        Self {
            start,
            goals,
            current: Cell::new(search_start),
            state: Cell::new(State::Solving),
            candidates: RefCell::new(Vec::new()),
            maze,
            _node_max: PhantomData,
        }
    }
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum SolverError {
    WaitingError,
    SolveFinishError,
}

impl TryInto<FinishError> for SolverError {
    type Error = ();

    fn try_into(self) -> Result<FinishError, Self::Error> {
        match self {
            Self::WaitingError => Err(()),
            Self::SolveFinishError => Ok(FinishError),
        }
    }
}

impl<Node, SearchNode, Max, GoalSize, Maze> Solver<Node, SearchNode, Max, GoalSize, Maze>
where
    GoalSize: ArrayLength<Node>,
{
    //helper function generating iterator of reference from candidates.
    //TODO: remove this function
    fn candidates_iter<'a, I>(candidates: &'a Vec<SearchNode, CandNum>) -> I
    where
        &'a Vec<SearchNode, CandNum>: IntoIterator<IntoIter = I, Item = &'a SearchNode>,
        I: Iterator<Item = &'a SearchNode>,
    {
        candidates.into_iter()
    }
}

impl<Node, Cost, Max, GoalSize, Maze, Obstacle> SearchCommander<Obstacle>
    for Solver<Node, Maze::SearchNode, Max, GoalSize, Maze>
where
    Max: ArrayLength<Node>
        + ArrayLength<Maze::SearchNode>
        + ArrayLength<Cost>
        + ArrayLength<Reverse<Cost>>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<(Maze::SearchNode, Reverse<Cost>)>
        + ArrayLength<Option<Node>>
        + ArrayLength<Option<usize>>
        + typenum::Unsigned,
    GoalSize: ArrayLength<Node>,
    Node: Ord + Copy + Debug + Into<usize>,
    Maze::SearchNode: Ord + Copy + Debug + Into<usize>,
    Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<Node, Cost = Cost>
        + Graph<<Maze as GraphConverter<Node>>::SearchNode, Cost = Cost>
        + GraphConverter<Node>
        + ObstacleInterpreter<Obstacle>
        + NodeChecker<<Maze as GraphConverter<Node>>::SearchNode>
        + Converter<<Maze as GraphConverter<Node>>::SearchNode, Target = Pose>
        + Converter<
            (
                <Maze as GraphConverter<Node>>::SearchNode,
                <Maze as GraphConverter<Node>>::SearchNode,
            ),
            Target = SearchKind,
        >,
    <Maze as Converter<<Maze as GraphConverter<Node>>::SearchNode>>::Error: core::fmt::Debug,
    <Maze as Converter<(
        <Maze as GraphConverter<Node>>::SearchNode,
        <Maze as GraphConverter<Node>>::SearchNode,
    )>>::Error: core::fmt::Debug,
{
    type Error = SolverError;
    type Command = (Pose, SearchKind);

    //TODO: write test
    fn update_obstacles<Obstacles: IntoIterator<Item = Obstacle>>(&self, obstacles: Obstacles) {
        self.maze.interpret_obstacles(obstacles);
    }

    //must return the current pose and next kind
    //TODO: write test
    fn next_command(&self) -> Result<Self::Command, Self::Error> {
        match self.state.get() {
            State::Solving => {
                let candidates = self
                    .next_node_candidates(&self.current.get())
                    .ok_or(SolverError::SolveFinishError)?;
                self.candidates.replace(candidates);
                self.state.set(State::Waiting);
                Err(SolverError::WaitingError)
            }
            State::Waiting => {
                use core::ops::Deref;

                let mut next = None;
                {
                    let candidates = self.candidates.borrow();
                    if candidates.is_empty() {
                        return Err(SolverError::WaitingError);
                    }
                    for &node in Self::candidates_iter(candidates.deref()) {
                        let is_available = self
                            .maze
                            .is_available(&node)
                            .map_err(|_| SolverError::WaitingError)?;
                        if is_available {
                            next = Some(node);
                            break;
                        }
                    }
                }
                self.candidates.replace(Vec::new());
                self.state.set(State::Solving);
                if let Some(next) = next {
                    let current = self.current.get();
                    let kind = self
                        .maze
                        .convert(&(current, next))
                        .unwrap_or_else(|err| unreachable!("This is bug: {:?}", err));
                    let pose = self
                        .maze
                        .convert(&current)
                        .unwrap_or_else(|err| unreachable!("This is bug: {:?}", err));
                    self.current.set(next);
                    Ok((pose, kind))
                } else {
                    Err(SolverError::SolveFinishError)
                }
            }
        }
    }
}

impl<Node, Cost, Max, GoalSize, Maze> Solver<Node, Maze::SearchNode, Max, GoalSize, Maze>
where
    Max: ArrayLength<Node>
        + ArrayLength<Maze::SearchNode>
        + ArrayLength<Cost>
        + ArrayLength<Reverse<Cost>>
        + ArrayLength<(Node, Reverse<Cost>)>
        + ArrayLength<(Maze::SearchNode, Reverse<Cost>)>
        + ArrayLength<Option<Node>>
        + ArrayLength<Option<usize>>
        + typenum::Unsigned,
    GoalSize: ArrayLength<Node>,
    Node: Ord + Copy + Debug + Into<usize>,
    Maze::SearchNode: Ord + Copy + Debug + Into<usize>,
    Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<Node, Cost = Cost>
        + Graph<<Maze as GraphConverter<Node>>::SearchNode, Cost = Cost>
        + GraphConverter<Node>,
{
    fn next_node_candidates(
        &self,
        current: &Maze::SearchNode,
    ) -> Option<Vec<Maze::SearchNode, U4>> {
        let shortest_path = self.compute_shortest_path()?;
        let checker_nodes = self.maze.convert_to_checker_nodes(shortest_path);

        let candidates = self
            .maze
            .successors(current)
            .into_iter()
            .collect::<Vec<(Maze::SearchNode, Cost), U4>>();

        let mut dists = repeat_n(Cost::max_value(), Max::USIZE).collect::<GenericArray<_, Max>>();
        let mut heap = BinaryHeap::<Maze::SearchNode, Reverse<Cost>, Max>::new();
        for node in checker_nodes {
            heap.push_or_update(node, Reverse(Cost::min_value()))
                .unwrap();
            dists[node.into()] = Cost::min_value();
        }
        while let Some((node, Reverse(cost))) = heap.pop() {
            if candidates.iter().any(|&(cand, _)| cand == node) {
                continue;
            }
            for (next, edge_cost) in self.maze.predecessors(&node) {
                let next_cost = cost.saturating_add(edge_cost);
                if dists[next.into()] > next_cost {
                    dists[next.into()] = next_cost;
                    heap.push_or_update(next, Reverse(next_cost)).unwrap();
                }
            }
        }

        let mut candidates = candidates
            .into_iter()
            .map(|(node, cost)| (node, cost.saturating_add(dists[node.into()])))
            .filter(|&(_, cost)| cost < Cost::max_value())
            .collect::<Vec<(Maze::SearchNode, Cost), U4>>();

        if candidates.is_empty() {
            return None;
        }

        candidates.sort_unstable_by_key(|elem| elem.1);
        Some(candidates.into_iter().map(|(node, _)| node).collect())
    }
}

//TODO: propagate each conversion errors
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RunCommanderError {
    UnreachableError,
    ConversionError,
}

///NOTO: multiple implementations of Converter<_> can exist for different Target,
///but we assume there is only an implementation.
//TODO: write test
impl<Node, SearchNode, Max, GoalSize, Maze> RunCommander
    for Solver<Node, SearchNode, Max, GoalSize, Maze>
where
    Max: ArrayLength<Node>
        + ArrayLength<Option<Node>>
        + ArrayLength<Option<usize>>
        + ArrayLength<Maze::Cost>
        + ArrayLength<Reverse<Maze::Cost>>
        + ArrayLength<(Node, Reverse<Maze::Cost>)>
        + ArrayLength<(
            <Maze as Converter<Node>>::Target,
            <Maze as Converter<(Node, Node)>>::Target,
        )> + typenum::Unsigned,
    GoalSize: ArrayLength<Node>,
    Node: Ord + Copy + Debug + Into<usize>,
    Maze::Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<Node> + Converter<(Node, Node)> + Converter<Node>,
{
    type Error = RunCommanderError;
    type Command = (
        <Maze as Converter<Node>>::Target,
        <Maze as Converter<(Node, Node)>>::Target,
    );
    type Commands = Vec<Self::Command, Max>;

    fn compute_commands(&self) -> Result<Self::Commands, Self::Error> {
        let path = self
            .compute_shortest_path()
            .ok_or(RunCommanderError::UnreachableError)?;

        let mut commands = Vec::new();
        if path.len() == 0 {
            return Ok(commands);
        }
        for i in 0..path.len() - 1 {
            commands
                .push((
                    self.maze
                        .convert(&path[i])
                        .map_err(|_| RunCommanderError::ConversionError)?,
                    self.maze
                        .convert(&(path[i], path[i + 1]))
                        .map_err(|_| RunCommanderError::ConversionError)?,
                ))
                .unwrap_or_else(|_| unreachable!());
        }
        Ok(commands)
    }
}

impl<Node, SearchNode, Max, GoalSize, Maze> Solver<Node, SearchNode, Max, GoalSize, Maze>
where
    Max: ArrayLength<Node>
        + ArrayLength<Option<Node>>
        + ArrayLength<Option<usize>>
        + ArrayLength<Maze::Cost>
        + ArrayLength<Reverse<Maze::Cost>>
        + ArrayLength<(Node, Reverse<Maze::Cost>)>
        + typenum::Unsigned,
    GoalSize: ArrayLength<Node>,
    Node: Ord + Copy + Debug + Into<usize>,
    Maze::Cost: Ord + Bounded + Saturating + num::Unsigned + Debug + Copy,
    Maze: Graph<Node>,
{
    pub fn compute_shortest_path(&self) -> Option<Vec<Node, Max>> {
        let mut dists =
            repeat_n(Maze::Cost::max_value(), Max::USIZE).collect::<GenericArray<_, Max>>();
        dists[self.start.into()] = Maze::Cost::min_value();

        let mut prev = repeat_n(None, Max::USIZE).collect::<GenericArray<Option<Node>, Max>>();

        let mut heap = BinaryHeap::<Node, Reverse<Maze::Cost>, Max>::new();
        heap.push(self.start, Reverse(Maze::Cost::min_value()))
            .unwrap();

        let construct_path = |goal: Node, prev: GenericArray<_, _>| {
            let mut current = prev[goal.into()]?;
            let mut path = Vec::new();
            path.push(goal).unwrap();
            path.push(current).unwrap();
            while let Some(next) = prev[current.into()] {
                path.push(next).unwrap();
                current = next;
            }
            let len = path.len();
            //reverse
            for i in 0..len / 2 {
                path.swap(i, len - i - 1);
            }
            Some(path)
        };

        while let Some((node, Reverse(cost))) = heap.pop() {
            for &goal in &self.goals {
                if node == goal {
                    return construct_path(goal, prev);
                }
            }
            for (next, edge_cost) in self.maze.successors(&node) {
                let next_cost = cost.saturating_add(edge_cost);
                if next_cost < dists[next.into()] {
                    dists[next.into()] = next_cost;
                    heap.push_or_update(next, Reverse(next_cost)).unwrap();
                    prev[next.into()] = Some(node);
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use alloc::rc::Rc;

    use generic_array::arr;
    use heapless::consts::*;

    use super::{Graph, Solver};
    use crate::maze::MazeBuilder;

    struct IGraph {
        n: usize,
        mat: Vec<Vec<Option<usize>>>,
    }

    impl IGraph {
        fn new(n: usize, edges: &[(usize, usize, usize)]) -> Self {
            let mut mat = vec![vec![None; n]; n];
            for &(src, dst, cost) in edges {
                mat[src][dst] = Some(cost);
            }
            Self { n, mat }
        }
    }

    impl Graph<usize> for IGraph {
        type Cost = usize;
        type Edges = Vec<(usize, usize)>;

        fn successors(&self, node: &usize) -> Self::Edges {
            let mut result = Vec::new();
            for i in 0..self.n {
                if let Some(cost) = self.mat[*node][i] {
                    result.push((i, cost));
                }
            }
            result
        }

        fn predecessors(&self, node: &usize) -> Self::Edges {
            let mut result = Vec::new();
            for i in 0..self.n {
                if let Some(cost) = self.mat[i][*node] {
                    result.push((i, cost));
                }
            }
            result
        }
    }

    #[test]
    fn test_compute_shortest_path() {
        use generic_array::arr;

        let edges = [
            (0, 1, 2),
            (0, 2, 1),
            (1, 3, 1),
            (2, 3, 3),
            (3, 4, 2),
            (3, 5, 5),
            (3, 6, 4),
            (5, 6, 3),
            (5, 7, 7),
            (7, 8, 1),
        ];
        let start = 0;
        let goals = arr![usize; 8];
        let n = 9;

        let graph = IGraph::new(n, &edges);

        let solver = Solver::<usize, usize, U9, _, _>::new(start, goals, start, Rc::new(graph));

        let path = solver.compute_shortest_path();
        let expected = [0, 1, 3, 5, 7, 8];

        assert!(path.is_some());
        assert_eq!(path.unwrap().as_ref(), expected);
    }

    macro_rules! next_node_candidates_tests {
        ($($name: ident: $value: expr,)*) => {
            $(
                #[test]
                fn $name() {
                    use generic_array::arr;
                    use crate::maze::{AbsoluteDirection::*, WallPosition, WallDirection::*, NodeId, SearchNodeId};
                    use crate::utils::math::MathFake;
                    use crate::data_types::Pattern;

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

                    let new = |x, y, dir| {
                        NodeId::<U4>::new(x, y, dir)
                            .unwrap_or_else(|err| unreachable!("invalid test parameter: {:?}", err))
                    };
                    let new_search = |x, y, dir| {
                        SearchNodeId::<U4>::new(x, y, dir)
                            .unwrap_or_else(|err| unreachable!("invalid test parameter: {:?}", err))
                    };
                    let new_wall = |x, y, z| {
                        WallPosition::<U4>::new(x, y, z)
                            .unwrap_or_else(|err| unreachable!("invalid test parameter: {:?}", err))
                    };

                    let start = new(0, 0, North);
                    let goals = arr![NodeId<U4>; new(2,0,West), new(2,0,South)];

                    let (input, expected) = $value;
                    let current = input.0;
                    let walls = input.1;

                    let current = new_search(current.0, current.1, current.2);
                    let maze = MazeBuilder::new().costs(cost).build::<U4, MathFake>();
                    for (wall, exists) in walls.into_iter().map(|wall| (new_wall(wall.0,wall.1,wall.2), wall.3)) {
                        maze.check_wall(wall, exists);
                    }
                    let expected = expected.into_iter().map(|input| new_search(input.0, input.1, input.2)).collect::<Vec<_>>();

                    let solver = Solver::<NodeId<U4>, SearchNodeId<U4>, U256, _, _>::new(start, goals, current, Rc::new(maze));
                    let candidates = solver.next_node_candidates(&current);
                    assert_eq!(candidates.expect("Failed to unwrap candidates"), expected.as_slice());
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
                (2,1,South),
                (2,3,North),
                (3,2,East),
                (1,2,West),
            ],
        ),
        test_next_node_candidates3: (
            (
                (6,5,South),
                vec![
                    (0,1,Right,true),
                    (0,1,Up,false),
                    (0,2,Right,true),
                    (0,2,Up,false),
                    (0,3,Right,false),
                    (1,2,Up,false),
                    (1,3,Right,false),
                    (2,2,Right,false),
                    (2,2,Up,true),
                    (2,3,Right,false),
                    (3,1,Up,true),
                    (3,2,Up,false),
                ],
            ),
            vec![
                (5,4,West),
                (6,5,North),
            ],
        ),
    }
}
