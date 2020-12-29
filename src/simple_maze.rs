use core::marker::PhantomData;

use heapless::{ArrayLength, Vec};
use uom::si::f32::Length;

use crate::commander::{CannotCheckError, Graph, GraphConverter, NodeChecker, ObstacleInterpreter};
use crate::obstacle_detector::Obstacle;
use crate::utils::{forced_vec::ForcedVec, math::Math, probability::Probability};

///This trait should be implemented as thread safe.
pub trait WallManager<Wall, State> {
    type Error;

    //State can take values such as boolean or float (probability)
    fn try_existence(&self, wall: &Wall) -> Result<State, Self::Error>; //existence state
    fn try_check(&self, wall: &Wall) -> Result<State, Self::Error>; //check state
    fn try_update(&self, wall: &Wall, state: &State) -> Result<(), Self::Error>; //update state

    fn existence(&self, wall: &Wall) -> State {
        loop {
            if let Ok(state) = self.try_existence(wall) {
                break state;
            }
        }
    }

    fn check(&self, wall: &Wall) -> State {
        loop {
            if let Ok(state) = self.try_check(wall) {
                break state;
            }
        }
    }

    fn update(&self, wall: &Wall, state: &State) {
        while self.try_update(wall, state).is_err() {}
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WallNode<Wall, Node> {
    Wall(Wall),
    Node(Node),
}

pub trait GraphNode: Sized {
    type Wall;
    type Cost;
    type WallNodes: IntoIterator<Item = WallNode<Self::Wall, (Self, Self::Cost)>>;
    type WallNodesList: IntoIterator<Item = Self::WallNodes>;

    fn successors(&self) -> Self::WallNodesList;
    fn predecessors(&self) -> Self::WallNodesList;
}

pub trait WallEnumerator {
    type Wall;
    type Walls: IntoIterator<Item = Self::Wall>;

    fn walls_between(&self, to: &Self) -> Self::Walls;
}

pub struct WallInfo<Wall> {
    pub wall: Wall,
    pub existing_distance: Length,
    pub not_existing_distance: Length,
}

pub trait ObstacleConverter {
    type Error;
    type Wall;

    fn convert(&self, obstacle: &Obstacle) -> Result<WallInfo<Self::Wall>, Self::Error>;
}

pub struct Maze<SearchNode, NeighborNum, SearchNodeNum, Manager, Converter, MathType> {
    manager: Manager,
    converter: Converter,
    _search_node: PhantomData<fn() -> SearchNode>,
    _math: PhantomData<fn() -> MathType>,
    _search_node_num: PhantomData<fn() -> SearchNodeNum>,
    _neighbor_num: PhantomData<fn() -> NeighborNum>,
}

impl<SearchNode, NeighborNum, SearchNodeNum, Manager, Converter, MathType>
    Maze<SearchNode, NeighborNum, SearchNodeNum, Manager, Converter, MathType>
{
    pub fn new(manager: Manager, converter: Converter) -> Self {
        Self {
            manager,
            converter,
            _search_node: PhantomData,
            _math: PhantomData,
            _search_node_num: PhantomData,
            _neighbor_num: PhantomData,
        }
    }
}

impl<Node, SearchNode, NeighborNum, SearchNodeNum, Manager, Converter, MathType> Graph<Node>
    for Maze<SearchNode, NeighborNum, SearchNodeNum, Manager, Converter, MathType>
where
    Node: GraphNode,
    Manager: WallManager<Node::Wall, bool>,
    NeighborNum: ArrayLength<(Node, Node::Cost)>,
{
    type Cost = Node::Cost;
    type Edges = Vec<(Node, Self::Cost), NeighborNum>;

    fn successors(&self, node: &Node) -> Self::Edges {
        let mut successors = ForcedVec::new();
        for wall_nodes in node.successors() {
            for wall_node in wall_nodes {
                match wall_node {
                    WallNode::Wall(wall) => {
                        if self.manager.existence(&wall) {
                            break;
                        }
                    }
                    WallNode::Node(node) => {
                        successors.push(node);
                    }
                }
            }
        }
        successors.into()
    }

    fn predecessors(&self, node: &Node) -> Self::Edges {
        let mut predecessors = ForcedVec::new();
        for wall_nodes in node.predecessors() {
            for wall_node in wall_nodes {
                match wall_node {
                    WallNode::Wall(wall) => {
                        if self.manager.existence(&wall) {
                            break;
                        }
                    }
                    WallNode::Node(node) => {
                        predecessors.push(node);
                    }
                }
            }
        }
        predecessors.into()
    }
}

//TODO: Write test
impl<Node, SearchNode, NeighborNum, SearchNodeNum, Manager, Converter, MathType>
    GraphConverter<Node>
    for Maze<SearchNode, NeighborNum, SearchNodeNum, Manager, Converter, MathType>
where
    Node: WallEnumerator,
    Node::Wall: Into<[SearchNode; 2]>,
    Manager: WallManager<Node::Wall, bool>,
    SearchNodeNum: ArrayLength<SearchNode>,
{
    type SearchNode = SearchNode;
    type SearchNodes = Vec<Self::SearchNode, SearchNodeNum>;

    fn convert_to_checker_nodes<Nodes: core::ops::Deref<Target = [Node]>>(
        &self,
        path: Nodes,
    ) -> Self::SearchNodes {
        let mut nodes = ForcedVec::new();
        for i in 0..path.len() - 1 {
            for wall in path[i].walls_between(&path[i + 1]) {
                if self.manager.check(&wall) {
                    continue;
                }
                let [node1, node2] = wall.into();
                nodes.push(node1);
                nodes.push(node2);
            }
        }
        nodes.into()
    }
}

//TODO: Write test
impl<SearchNode, NeighborNum, SearchNodeNum, Manager, Converter, MathType> NodeChecker<SearchNode>
    for Maze<SearchNode, NeighborNum, SearchNodeNum, Manager, Converter, MathType>
where
    SearchNode: GraphNode + Into<<SearchNode as GraphNode>::Wall> + Clone,
    Manager: WallManager<SearchNode::Wall, bool>,
{
    fn is_available(&self, node: &SearchNode) -> Result<bool, CannotCheckError> {
        let wall = node.clone().into();
        if let Ok(check) = self.manager.try_check(&wall) {
            if check {
                if let Ok(existence) = self.manager.try_existence(&wall) {
                    return Ok(!existence);
                }
            }
        }
        Err(CannotCheckError)
    }
}

//TODO: Write test
impl<SearchNode, NeighborNum, SearchNodeNum, Manager, Converter, MathType>
    ObstacleInterpreter<Obstacle>
    for Maze<SearchNode, NeighborNum, SearchNodeNum, Manager, Converter, MathType>
where
    Converter: ObstacleConverter,
    Manager: WallManager<Converter::Wall, Probability>,
    MathType: Math,
{
    type Error = core::cell::BorrowMutError;

    fn interpret_obstacles<Obstacles: IntoIterator<Item = Obstacle>>(
        &self,
        obstacles: Obstacles,
    ) -> Result<(), Self::Error> {
        use uom::si::ratio::ratio;

        for obstacle in obstacles {
            if let Ok(wall_info) = self.converter.convert(&obstacle) {
                if let Ok(existence) = self.manager.try_existence(&wall_info.wall) {
                    let exist_val = {
                        let tmp = ((wall_info.existing_distance - obstacle.distance.mean)
                            / obstacle.distance.standard_deviation)
                            .get::<ratio>();
                        -tmp * tmp / 2.0
                    };
                    let not_exist_val = {
                        let tmp = ((wall_info.not_existing_distance - obstacle.distance.mean)
                            / obstacle.distance.standard_deviation)
                            .get::<ratio>();
                        -tmp * tmp / 2.0
                    };

                    let min = if exist_val < not_exist_val {
                        exist_val
                    } else {
                        not_exist_val
                    };
                    let exist_val = MathType::expf(exist_val - min) * existence;
                    let not_exist_val = MathType::expf(not_exist_val - min) * existence.reverse();

                    let existence = if exist_val.is_infinite() {
                        Probability::one()
                    } else if not_exist_val.is_infinite() {
                        Probability::zero()
                    } else {
                        Probability::new(exist_val / (exist_val + not_exist_val)).unwrap_or_else(
                            |err| unreachable!("Should never be out of bound: {:?}", err),
                        )
                    };
                    let _ = self.manager.try_update(&wall_info.wall, &existence);
                    //ignore if cannot update
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use heapless::consts::*;

    use super::*;

    #[test]
    fn test_graph() {
        use std::vec::Vec;

        impl GraphNode for usize {
            type Wall = usize;
            type Cost = usize;
            type WallNodes = Vec<WallNode<usize, (usize, usize)>>;
            type WallNodesList = Vec<Self::WallNodes>;

            fn successors(&self) -> Self::WallNodesList {
                let mut res = Vec::new();
                let mut tmp = Vec::new();
                tmp.push(WallNode::Wall(0));
                tmp.push(WallNode::Node((2, 2)));
                tmp.push(WallNode::Wall(1));
                tmp.push(WallNode::Wall(2));
                tmp.push(WallNode::Node((5, 3)));
                res.push(tmp);

                let mut tmp = Vec::new();
                tmp.push(WallNode::Wall(3));
                tmp.push(WallNode::Node((2, 1)));
                tmp.push(WallNode::Wall(4));
                tmp.push(WallNode::Node((4, 2)));
                tmp.push(WallNode::Wall(5));
                tmp.push(WallNode::Node((6, 3)));
                res.push(tmp);
                res
            }

            fn predecessors(&self) -> Self::WallNodesList {
                self.successors()
            }
        }

        struct WallManagerType;

        impl WallManager<usize, bool> for WallManagerType {
            type Error = core::convert::Infallible;

            fn try_existence(&self, wall: &usize) -> Result<bool, Self::Error> {
                match wall {
                    2 | 5 => Ok(true),
                    _ => Ok(false),
                }
            }

            fn try_check(&self, _wall: &usize) -> Result<bool, Self::Error> {
                unimplemented!()
            }

            fn try_update(&self, _wall: &usize, _state: &bool) -> Result<(), Self::Error> {
                unimplemented!()
            }
        }

        let manager = WallManagerType;

        let maze = Maze::<(), U10, (), _, (), ()>::new(manager, ());
        let expected = vec![(2usize, 2usize), (2, 1), (4, 2)];
        assert_eq!(maze.successors(&0), expected.as_slice());
        assert_eq!(maze.predecessors(&0), expected.as_slice());
    }
}
