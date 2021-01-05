mod direction;

use core::marker::PhantomData;

use heapless::{ArrayLength, Vec};
use uom::si::f32::Length;

use crate::commander::{CannotCheckError, Graph, GraphConverter, NodeChecker, ObstacleInterpreter};
use crate::data_types::Pose;
use crate::obstacle_detector::Obstacle;
use crate::utils::{forced_vec::ForcedVec, math::Math, probability::Probability};
pub use direction::{AbsoluteDirection, RelativeDirection};

macro_rules! block {
    ($expr: expr) => {
        loop {
            if let Ok(val) = $expr {
                break val;
            }
        }
    };
}

///This trait should be implemented as thread safe.
pub trait WallManager<Wall> {
    type Error;

    fn try_existence_probability(&self, wall: &Wall) -> Result<Probability, Self::Error>;
    fn try_update(&self, wall: &Wall, probablity: &Probability) -> Result<(), Self::Error>;

    #[inline]
    fn existence_threshold(&self) -> Probability {
        Probability::zero()
    }

    fn try_is_checked(&self, wall: &Wall) -> Result<bool, Self::Error> {
        let prob = self.try_existence_probability(wall)?;
        Ok(prob < self.existence_threshold() || prob > self.existence_threshold().reverse())
    }

    fn try_exists(&self, wall: &Wall) -> Result<bool, Self::Error> {
        let prob = self.try_existence_probability(wall)?;
        Ok(prob > self.existence_threshold().reverse())
    }

    fn existence_probablity(&self, wall: &Wall) -> Probability {
        block!(self.try_existence_probability(wall))
    }

    fn is_checked(&self, wall: &Wall) -> bool {
        block!(self.try_is_checked(wall))
    }

    fn exists(&self, wall: &Wall) -> bool {
        block!(self.try_exists(wall))
    }

    fn update(&self, wall: &Wall, state: &Probability) {
        block!(self.try_update(wall, state))
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WallNode<Wall, Node> {
    Wall(Wall),
    Node(Node),
}

pub trait WallSpaceNode {
    type Wall;
}

pub trait BoundedNode {
    type NodeNum;
}

pub trait GraphNode: Sized + WallSpaceNode {
    type NeighborNum;
    type Cost;
    type WallNodes: IntoIterator<Item = WallNode<Self::Wall, (Self, Self::Cost)>>;
    type WallNodesList: IntoIterator<Item = Self::WallNodes>;

    fn successors(&self) -> Self::WallNodesList;
    fn predecessors(&self) -> Self::WallNodesList;
}

pub trait WallFinderNode: WallSpaceNode {
    type Walls: IntoIterator<Item = Self::Wall>;

    fn walls_between(&self, other: &Self) -> Self::Walls;
}

pub struct WallInfo<Wall> {
    pub wall: Wall,
    pub existing_distance: Length,
    pub not_existing_distance: Length,
}

pub trait PoseConverter<Pose> {
    type Error;
    type Wall;

    fn convert(&self, pose: &Pose) -> Result<WallInfo<Self::Wall>, Self::Error>;
}

pub trait WallConverter<Wall> {
    type Error;
    type SearchNode;
    type SearchNodes: IntoIterator<Item = Self::SearchNode>;

    fn convert(&self, wall: &Wall) -> Result<Self::SearchNodes, Self::Error>;
}

pub struct Maze<Manager, PoseConverterType, WallConverterType, MathType> {
    manager: Manager,
    pose_converter: PoseConverterType,
    wall_converter: WallConverterType,
    _math: PhantomData<fn() -> MathType>,
}

impl<Manager, PoseConverterType, WallConverterType, MathType> core::fmt::Debug
    for Maze<Manager, PoseConverterType, WallConverterType, MathType>
where
    Manager: core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.manager.fmt(f)
    }
}

impl<Manager, PoseConverterType, WallConverterType, MathType>
    Maze<Manager, PoseConverterType, WallConverterType, MathType>
{
    pub fn new(
        manager: Manager,
        obstacle_converter: PoseConverterType,
        wall_converter: WallConverterType,
    ) -> Self {
        Self {
            manager,
            pose_converter: obstacle_converter,
            wall_converter,
            _math: PhantomData,
        }
    }
}

impl<Node, Manager, PoseConverterType, WallConverterType, MathType> Graph<Node>
    for Maze<Manager, PoseConverterType, WallConverterType, MathType>
where
    Node: GraphNode,
    Manager: WallManager<Node::Wall>,
    Node::NeighborNum: ArrayLength<(Node, Node::Cost)>,
{
    type Cost = Node::Cost;
    type Edges = Vec<(Node, Self::Cost), Node::NeighborNum>;

    fn successors(&self, node: &Node) -> Self::Edges {
        let mut successors = ForcedVec::new();
        for wall_nodes in node.successors() {
            for wall_node in wall_nodes {
                match wall_node {
                    WallNode::Wall(wall) => {
                        if self.manager.exists(&wall) {
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
                        if self.manager.exists(&wall) {
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

impl<Node, Manager, PoseConverterType, WallConverterType, MathType> GraphConverter<Node>
    for Maze<Manager, PoseConverterType, WallConverterType, MathType>
where
    Node: WallFinderNode,
    WallConverterType: WallConverter<Node::Wall>,
    WallConverterType::SearchNode: BoundedNode,
    Manager: WallManager<Node::Wall>,
    <WallConverterType::SearchNode as BoundedNode>::NodeNum:
        ArrayLength<WallConverterType::SearchNode>,
{
    type SearchNode = WallConverterType::SearchNode;
    type SearchNodes =
        Vec<Self::SearchNode, <WallConverterType::SearchNode as BoundedNode>::NodeNum>;

    fn convert_to_checker_nodes<Nodes: core::ops::Deref<Target = [Node]>>(
        &self,
        path: Nodes,
    ) -> Self::SearchNodes {
        let mut nodes = ForcedVec::new();
        for i in 0..path.len() - 1 {
            for wall in path[i].walls_between(&path[i + 1]) {
                if self.manager.is_checked(&wall) {
                    continue;
                }
                if let Ok(wall_nodes) = self.wall_converter.convert(&wall) {
                    for node in wall_nodes {
                        nodes.push(node);
                    }
                }
            }
        }
        nodes.into()
    }
}

impl<SearchNode, Manager, PoseConverterType, WallConverterType, MathType> NodeChecker<SearchNode>
    for Maze<Manager, PoseConverterType, WallConverterType, MathType>
where
    SearchNode: WallSpaceNode + Into<<SearchNode as WallSpaceNode>::Wall> + Clone,
    Manager: WallManager<SearchNode::Wall>,
{
    fn is_available(&self, node: &SearchNode) -> Result<bool, CannotCheckError> {
        let wall = node.clone().into();
        if let Ok(check) = self.manager.try_is_checked(&wall) {
            if check {
                if let Ok(existence) = self.manager.try_exists(&wall) {
                    return Ok(!existence);
                }
            }
        }
        Err(CannotCheckError)
    }
}

//TODO: Write test
impl<Manager, PoseConverterType, WallConverterType, MathType> ObstacleInterpreter<Obstacle>
    for Maze<Manager, PoseConverterType, WallConverterType, MathType>
where
    PoseConverterType: PoseConverter<Pose>,
    Manager: WallManager<PoseConverterType::Wall>,
    MathType: Math,
{
    type Error = core::cell::BorrowMutError;

    fn interpret_obstacles<Obstacles: IntoIterator<Item = Obstacle>>(
        &self,
        obstacles: Obstacles,
    ) -> Result<(), Self::Error> {
        use uom::si::ratio::ratio;

        for obstacle in obstacles {
            if let Ok(wall_info) = self.pose_converter.convert(&obstacle.source) {
                if let Ok(existence) = self.manager.try_existence_probability(&wall_info.wall) {
                    if existence.is_zero() || existence.is_one() {
                        continue;
                    }

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

                    debug_assert!(!exist_val.is_nan());
                    debug_assert!(!not_exist_val.is_nan());

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

        impl WallSpaceNode for usize {
            type Wall = usize;
        }

        impl GraphNode for usize {
            type NeighborNum = U10;
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

        impl WallManager<usize> for WallManagerType {
            type Error = core::convert::Infallible;

            fn try_existence_probability(&self, _wall: &usize) -> Result<Probability, Self::Error> {
                unimplemented!()
            }

            fn try_update(&self, _wall: &usize, _prob: &Probability) -> Result<(), Self::Error> {
                unimplemented!()
            }

            fn try_exists(&self, wall: &usize) -> Result<bool, Self::Error> {
                match wall {
                    2 | 5 => Ok(true),
                    _ => Ok(false),
                }
            }

            fn try_is_checked(&self, _wall: &usize) -> Result<bool, Self::Error> {
                unimplemented!()
            }
        }

        let manager = WallManagerType;

        let maze = Maze::<_, (), (), ()>::new(manager, (), ());
        let expected = vec![(2usize, 2usize), (2, 1), (4, 2)];
        assert_eq!(maze.successors(&0), expected.as_slice());
        assert_eq!(maze.predecessors(&0), expected.as_slice());
    }

    #[test]
    fn test_graph_converter() {
        use std::vec::Vec;

        #[derive(Debug, PartialEq, Eq, Clone, Copy)]
        struct Node(usize);

        #[derive(Debug, PartialEq, Eq, Clone, Copy)]
        struct SearchNode(usize);

        #[derive(Debug, PartialEq, Eq, Clone, Copy)]
        struct Wall(usize);

        impl WallSpaceNode for Node {
            type Wall = Wall;
        }

        impl BoundedNode for SearchNode {
            type NodeNum = U10;
        }

        impl WallFinderNode for Node {
            type Walls = Vec<Self::Wall>;

            fn walls_between(&self, other: &Self) -> Self::Walls {
                if self.0 % 2 == 0 {
                    vec![Wall(self.0), Wall(other.0)]
                } else {
                    vec![Wall(other.0)]
                }
            }
        }

        impl Into<[SearchNode; 2]> for Wall {
            fn into(self) -> [SearchNode; 2] {
                [SearchNode(self.0), SearchNode(self.0 + 1)]
            }
        }

        struct WallConverterType;

        impl WallConverter<Wall> for WallConverterType {
            type Error = core::convert::Infallible;
            type SearchNode = SearchNode;
            type SearchNodes = Vec<Self::SearchNode>;

            fn convert(&self, wall: &Wall) -> Result<Self::SearchNodes, Self::Error> {
                Ok(vec![SearchNode(wall.0), SearchNode(wall.0 + 1)])
            }
        }

        struct WallManagerType;

        impl WallManager<Wall> for WallManagerType {
            type Error = core::convert::Infallible;

            fn try_existence_probability(&self, _wall: &Wall) -> Result<Probability, Self::Error> {
                unimplemented!()
            }

            fn try_is_checked(&self, wall: &Wall) -> Result<bool, Self::Error> {
                Ok(wall.0 % 2 == 1)
            }

            fn try_update(&self, _wall: &Wall, _state: &Probability) -> Result<(), Self::Error> {
                unimplemented!()
            }
        }

        let maze = Maze::<_, (), _, ()>::new(WallManagerType, (), WallConverterType);

        let path = vec![0, 1, 1, 2, 3, 5, 8]
            .into_iter()
            .map(|e| Node(e))
            .collect::<Vec<_>>();
        let expected = vec![0, 1, 2, 3, 2, 3, 8, 9]
            .into_iter()
            .map(|e| SearchNode(e))
            .collect::<Vec<_>>();
        assert_eq!(maze.convert_to_checker_nodes(path), expected.as_slice());
    }

    #[test]
    fn test_is_available() {
        struct WallManagerType;

        impl WallManager<usize> for WallManagerType {
            type Error = core::convert::Infallible;

            fn try_existence_probability(&self, _wall: &usize) -> Result<Probability, Self::Error> {
                unimplemented!()
            }

            fn try_exists(&self, wall: &usize) -> Result<bool, Self::Error> {
                Ok(match wall {
                    0 | 3 => true,
                    _ => false,
                })
            }

            fn try_is_checked(&self, wall: &usize) -> Result<bool, Self::Error> {
                Ok(match wall {
                    0 | 1 | 3 | 5 => true,
                    _ => false,
                })
            }

            fn try_update(&self, _wall: &usize, _state: &Probability) -> Result<(), Self::Error> {
                unimplemented!()
            }
        }

        let maze = Maze::<_, (), (), ()>::new(WallManagerType, (), ());
        let test_cases = vec![
            (0, Ok(false)),
            (1, Ok(true)),
            (2, Err(CannotCheckError)),
            (3, Ok(false)),
        ];
        for (node, expected) in test_cases {
            assert_eq!(maze.is_available(&node), expected);
        }
    }
}
