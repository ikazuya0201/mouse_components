mod direction;

use core::fmt;

use heapless::{ArrayLength, Vec};

use crate::commanders::{BoundedNode, CannotCheckError, Graph, GraphConverter, NodeChecker};
use crate::utils::forced_vec::ForcedVec;
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
pub trait WallChecker<Wall>: Send + Sync {
    type Error;

    fn try_is_checked(&self, wall: &Wall) -> Result<bool, Self::Error>;
    fn try_exists(&self, wall: &Wall) -> Result<bool, Self::Error>;

    fn is_checked(&self, wall: &Wall) -> bool {
        block!(self.try_is_checked(wall))
    }

    fn exists(&self, wall: &Wall) -> bool {
        block!(self.try_exists(wall))
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

pub trait WallConverter<Wall> {
    type Error;
    type SearchNode;
    type SearchNodes: IntoIterator<Item = Self::SearchNode>;

    fn convert(&self, wall: &Wall) -> Result<Self::SearchNodes, Self::Error>;
}

pub struct Maze<'a, Manager, WallConverterType> {
    manager: &'a Manager,
    wall_converter: WallConverterType,
}

impl<'a, Manager, WallConverterType> fmt::Debug for Maze<'a, Manager, WallConverterType>
where
    Manager: fmt::Debug,
    WallConverterType: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Maze")
            .field("manager", &self.manager)
            .field("wall_converter", &self.wall_converter)
            .finish()
    }
}

impl<'a, Manager, WallConverterType> fmt::Display for Maze<'a, Manager, WallConverterType>
where
    Manager: fmt::Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.manager.fmt(f)
    }
}

impl<'a, Manager, WallConverterType> Maze<'a, Manager, WallConverterType> {
    pub fn new(manager: &'a Manager, wall_converter: WallConverterType) -> Self {
        Self {
            manager,
            wall_converter,
        }
    }
}

impl<'a, Node, Manager, WallConverterType> Graph<Node> for Maze<'a, Manager, WallConverterType>
where
    Node: GraphNode,
    Manager: WallChecker<Node::Wall>,
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

impl<'a, Node, Manager, WallConverterType> GraphConverter<Node>
    for Maze<'a, Manager, WallConverterType>
where
    Node: WallFinderNode,
    WallConverterType: WallConverter<Node::Wall>,
    WallConverterType::SearchNode: BoundedNode,
    Manager: WallChecker<Node::Wall>,
    <WallConverterType::SearchNode as BoundedNode>::UpperBound:
        ArrayLength<WallConverterType::SearchNode>,
{
    type SearchNode = WallConverterType::SearchNode;
    type SearchNodes =
        Vec<Self::SearchNode, <WallConverterType::SearchNode as BoundedNode>::UpperBound>;

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

impl<'a, SearchNode, Manager, WallConverterType> NodeChecker<SearchNode>
    for Maze<'a, Manager, WallConverterType>
where
    SearchNode: WallSpaceNode + Into<<SearchNode as WallSpaceNode>::Wall> + Clone,
    Manager: WallChecker<SearchNode::Wall>,
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

        impl WallChecker<usize> for WallManagerType {
            type Error = core::convert::Infallible;

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

        let maze = Maze::new(&manager, ());
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
            type UpperBound = U10;
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

        impl WallChecker<Wall> for WallManagerType {
            type Error = core::convert::Infallible;

            fn try_is_checked(&self, wall: &Wall) -> Result<bool, Self::Error> {
                Ok(wall.0 % 2 == 1)
            }

            fn try_exists(&self, _wall: &Wall) -> Result<bool, Self::Error> {
                unimplemented!()
            }
        }

        let manager = WallManagerType;
        let maze = Maze::new(&manager, WallConverterType);

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

        impl WallChecker<usize> for WallManagerType {
            type Error = core::convert::Infallible;

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
        }

        let manager = WallManagerType;

        let maze = Maze::new(&manager, ());
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
