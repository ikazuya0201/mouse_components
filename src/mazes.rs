//! Definition of [Maze](crate::mazes::Maze) and its dependent traits.

use alloc::rc::Rc;
use core::fmt;
use core::marker::PhantomData;

use heapless::Vec;

use crate::commanders::{
    GeometricGraph, Graph, NodeChecker, UncheckedNodeFinder, NODE_NUMBER_UPPER_BOUND,
};
use crate::utils::forced_vec::ForcedVec;
use crate::MAZE_WIDTH_UPPER_BOUND;
use crate::{Construct, Deconstruct};

macro_rules! block {
    ($expr: expr) => {
        loop {
            if let Ok(val) = $expr {
                break val;
            }
        }
    };
}

/// A trait that checks states of walls.
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

/// An enum that represents a wall or node.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WallNode<Wall, Node> {
    Wall(Wall),
    Node(Node),
}

/// A trait that specified the corresponding wall of a node.
pub trait WallSpaceNode {
    type Wall;
}

pub(crate) const NEIGHBOR_NUMBER_UPPER_BOUND: usize = 4 * MAZE_WIDTH_UPPER_BOUND;

/// A trait that enumerates wall-node successors and predecessors of a node.
pub trait GraphNode: Sized + WallSpaceNode {
    type Pattern;
    type WallNodes: IntoIterator<Item = WallNode<Self::Wall, (Self, Self::Pattern)>>;
    type WallNodesList: IntoIterator<Item = Self::WallNodes>;

    fn successors(&self) -> Self::WallNodesList;
    fn predecessors(&self) -> Self::WallNodesList;
}

/// A trait that enumerates walls between two nodes.
pub trait WallFinderNode: WallSpaceNode {
    type Walls: IntoIterator<Item = Self::Wall>;

    fn walls_between(&self, other: &Self) -> Self::Walls;
}

/// A trait that converts a pattern into a cost value.
pub trait PatternConverter<Pattern> {
    type Cost;

    fn convert(&self, pattern: &Pattern) -> Self::Cost;
}

/// An implementation of [Graph](crate::commanders::Graph),
/// [UncheckedNodeFinder](crate::commanders::UncheckedNodeFinder) and
/// [NodeChecker](crate::commanders::NodeChecker) required by
/// [SearchCommander](crate::commanders::SearchCommander).
pub struct Maze<Manager, Converter, SearchNode> {
    manager: Rc<Manager>,
    converter: Converter,
    _search_node: PhantomData<fn() -> SearchNode>,
}

impl<Manager, Converter, SearchNode> fmt::Debug for Maze<Manager, Converter, SearchNode>
where
    Manager: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Maze")
            .field("manager", &self.manager)
            .finish()
    }
}

impl<Manager, Converter, SearchNode> fmt::Display for Maze<Manager, Converter, SearchNode>
where
    Manager: fmt::Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.manager.fmt(f)
    }
}

impl<Manager, Converter, SearchNode> Maze<Manager, Converter, SearchNode> {
    pub fn new(manager: Rc<Manager>, converter: Converter) -> Self {
        Self {
            manager,
            converter,
            _search_node: PhantomData,
        }
    }

    pub fn release(self) -> Rc<Manager> {
        let Self { manager, .. } = self;
        manager
    }
}

impl<Manager, Converter, SearchNode, Config, State, Resource> Construct<Config, State, Resource>
    for Maze<Manager, Converter, SearchNode>
where
    Converter: Construct<Config, State, Resource>,
    Resource: AsRef<Rc<Manager>>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let converter = Converter::construct(config, state, resource);
        let wall_manager = Rc::clone(resource.as_ref());
        Self::new(wall_manager, converter)
    }
}

impl<Manager, Converter, SearchNode, State, Resource> Deconstruct<State, Resource>
    for Maze<Manager, Converter, SearchNode>
where
    Resource: From<Rc<Manager>>,
    State: Default,
{
    fn deconstruct(self) -> (State, Resource) {
        let manager = self.release();
        (State::default(), manager.into())
    }
}

impl<Manager, Converter, SearchNode> Maze<Manager, Converter, SearchNode> {
    fn _successors<Node, F>(&self, node: &Node, is_blocked: F) -> <Self as Graph<Node>>::Edges
    where
        F: Fn(&Node::Wall) -> bool,
        Node: GraphNode,
        Converter: PatternConverter<Node::Pattern>,
        Manager: WallChecker<Node::Wall>,
    {
        let mut successors = ForcedVec::new();
        for wall_nodes in node.successors() {
            for wall_node in wall_nodes {
                match wall_node {
                    WallNode::Wall(wall) => {
                        if is_blocked(&wall) {
                            break;
                        }
                    }
                    WallNode::Node((node, pattern)) => {
                        successors.push((node, self.converter.convert(&pattern)));
                    }
                }
            }
        }
        successors.into()
    }

    fn _predecessors<Node, F>(&self, node: &Node, is_blocked: F) -> <Self as Graph<Node>>::Edges
    where
        F: Fn(&Node::Wall) -> bool,
        Node: GraphNode,
        Converter: PatternConverter<Node::Pattern>,
        Manager: WallChecker<Node::Wall>,
    {
        let mut predecessors = ForcedVec::new();
        for wall_nodes in node.predecessors() {
            for wall_node in wall_nodes {
                match wall_node {
                    WallNode::Wall(wall) => {
                        if is_blocked(&wall) {
                            break;
                        }
                    }
                    WallNode::Node((node, pattern)) => {
                        predecessors.push((node, self.converter.convert(&pattern)));
                    }
                }
            }
        }
        predecessors.into()
    }
}

impl<Node, Manager, Converter, SearchNode> Graph<Node> for Maze<Manager, Converter, SearchNode>
where
    Node: GraphNode,
    Converter: PatternConverter<Node::Pattern>,
    Manager: WallChecker<Node::Wall>,
{
    type Cost = Converter::Cost;
    type Edges = Vec<(Node, Self::Cost), NEIGHBOR_NUMBER_UPPER_BOUND>;

    fn successors(&self, node: &Node) -> Self::Edges {
        //Define blocked state as existence of wall (doesn't consider whether a wall is checked or not).
        self._successors(node, |wall| self.manager.exists(wall))
    }

    fn predecessors(&self, node: &Node) -> Self::Edges {
        //Define blocked state as existence of wall (doesn't consider whether a wall is checked or not).
        self._predecessors(node, |wall| self.manager.exists(wall))
    }
}

/// A trait that represents a geometric node.
pub trait GeometricNode {
    type Output;

    fn distance(&self, other: &Self) -> Self::Output;
}

impl<Node, Manager, Converter, SearchNode> GeometricGraph<Node>
    for Maze<Manager, Converter, SearchNode>
where
    Node: GraphNode + GeometricNode<Output = Node::Pattern>,
    Converter: PatternConverter<Node::Pattern>,
    Manager: WallChecker<Node::Wall>,
{
    fn distance(&self, lhs: &Node, rhs: &Node) -> Self::Cost {
        self.converter.convert(&lhs.distance(rhs))
    }
}

impl<Node, Manager, Converter, SearchNode> UncheckedNodeFinder<Node>
    for Maze<Manager, Converter, SearchNode>
where
    Node: WallFinderNode,
    Manager: WallChecker<Node::Wall>,
    Node::Wall: Into<[SearchNode; 2]>,
{
    type SearchNode = SearchNode;
    type SearchNodes = Vec<Self::SearchNode, NODE_NUMBER_UPPER_BOUND>;

    fn find_unchecked_nodes(&self, path: &[Node]) -> Self::SearchNodes {
        let mut nodes = ForcedVec::new();
        for i in 0..path.len() - 1 {
            for wall in path[i].walls_between(&path[i + 1]) {
                if self.manager.is_checked(&wall) {
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

impl<Manager, Converter, SearchNode> NodeChecker<SearchNode>
    for Maze<Manager, Converter, SearchNode>
where
    SearchNode: WallSpaceNode + Into<<SearchNode as WallSpaceNode>::Wall> + Clone,
    Manager: WallChecker<SearchNode::Wall>,
{
    fn is_available(&self, node: &SearchNode) -> Option<bool> {
        let wall = node.clone().into();
        if let Ok(check) = self.manager.try_is_checked(&wall) {
            if check {
                if let Ok(existence) = self.manager.try_exists(&wall) {
                    return Some(!existence);
                }
            }
        }
        None
    }
}

/// A derivation of [Maze](Maze).
///
/// The implementation of [Graph](crate::commanders::Graph) is deffered from [Maze](Maze).
///
/// This checks whether a wall is checked or not in the implementation of [Graph](crate::commanders::Graph).
pub struct CheckedMaze<Manager, Converter, SearchNode>(Maze<Manager, Converter, SearchNode>);

impl<Manager, Converter, SearchNode> CheckedMaze<Manager, Converter, SearchNode> {
    pub fn new(manager: Rc<Manager>, converter: Converter) -> Self {
        Self(<Self as core::ops::Deref>::Target::new(manager, converter))
    }
}

impl<Manager, Converter, SearchNode, Config, State, Resource> Construct<Config, State, Resource>
    for CheckedMaze<Manager, Converter, SearchNode>
where
    Converter: Construct<Config, State, Resource>,
    Resource: AsRef<Rc<Manager>>,
{
    fn construct<'a>(config: &'a Config, state: &'a State, resource: &'a mut Resource) -> Self {
        let converter = Converter::construct(config, state, resource);
        let wall_manager = Rc::clone(resource.as_ref());
        Self::new(wall_manager, converter)
    }
}

impl<Manager, Converter, SearchNode, State, Resource> Deconstruct<State, Resource>
    for CheckedMaze<Manager, Converter, SearchNode>
where
    Resource: From<Rc<Manager>>,
    State: Default,
{
    fn deconstruct(self) -> (State, Resource) {
        self.0.deconstruct()
    }
}

impl<Manager, Converter, SearchNode> core::ops::Deref
    for CheckedMaze<Manager, Converter, SearchNode>
{
    type Target = Maze<Manager, Converter, SearchNode>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<Node, Manager, Converter, SearchNode> Graph<Node>
    for CheckedMaze<Manager, Converter, SearchNode>
where
    Node: GraphNode,
    Converter: PatternConverter<Node::Pattern>,
    Manager: WallChecker<Node::Wall>,
{
    type Cost = Converter::Cost;
    type Edges = Vec<(Node, Self::Cost), NEIGHBOR_NUMBER_UPPER_BOUND>;

    fn successors(&self, node: &Node) -> Self::Edges {
        //Define the blocked state as a state where a wall is not checked or exists.
        self._successors(node, |wall| {
            !self.manager.is_checked(wall) || self.manager.exists(wall)
        })
    }

    fn predecessors(&self, node: &Node) -> Self::Edges {
        //Define the blocked state as a state where a wall is not checked or exists.
        self._predecessors(node, |wall| {
            !self.manager.is_checked(wall) || self.manager.exists(wall)
        })
    }
}

impl<Node, Manager, Converter, SearchNode> GeometricGraph<Node>
    for CheckedMaze<Manager, Converter, SearchNode>
where
    Node: GraphNode + GeometricNode<Output = Node::Pattern>,
    Converter: PatternConverter<Node::Pattern>,
    Manager: WallChecker<Node::Wall>,
{
    #[inline]
    fn distance(&self, lhs: &Node, rhs: &Node) -> Self::Cost {
        self.0.distance(lhs, rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct ThroughPatternConverter;

    impl<N: Copy> PatternConverter<N> for ThroughPatternConverter {
        type Cost = N;

        fn convert(&self, pattern: &N) -> Self::Cost {
            *pattern
        }
    }

    #[test]
    fn test_graph() {
        use std::vec::Vec;

        impl WallSpaceNode for usize {
            type Wall = usize;
        }

        impl GraphNode for usize {
            type Pattern = usize;
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

        let maze = Maze::<_, _, usize>::new(Rc::new(manager), ThroughPatternConverter);
        let expected = vec![(2usize, 2usize), (2, 1), (4, 2)];
        assert_eq!(maze.successors(&0), expected.as_slice());
        assert_eq!(maze.predecessors(&0), expected.as_slice());
    }

    #[test]
    fn test_checked_maze_graph() {
        use std::vec::Vec;

        #[derive(Debug, PartialEq, Eq)]
        struct NodeType(usize);

        struct WallType(usize);

        impl WallSpaceNode for NodeType {
            type Wall = WallType;
        }

        impl GraphNode for NodeType {
            type Pattern = usize;
            type WallNodes = Vec<WallNode<WallType, (NodeType, usize)>>;
            type WallNodesList = Vec<Self::WallNodes>;

            fn successors(&self) -> Self::WallNodesList {
                let mut res = Vec::new();
                let mut tmp = Vec::new();
                tmp.push(WallNode::Wall(WallType(0)));
                tmp.push(WallNode::Node((NodeType(2), 2)));
                tmp.push(WallNode::Wall(WallType(1)));
                tmp.push(WallNode::Wall(WallType(2)));
                tmp.push(WallNode::Node((NodeType(5), 3)));
                res.push(tmp);

                let mut tmp = Vec::new();
                tmp.push(WallNode::Wall(WallType(3)));
                tmp.push(WallNode::Node((NodeType(2), 1)));
                tmp.push(WallNode::Wall(WallType(4)));
                tmp.push(WallNode::Node((NodeType(4), 2)));
                tmp.push(WallNode::Wall(WallType(5)));
                tmp.push(WallNode::Node((NodeType(6), 3)));
                res.push(tmp);
                res
            }

            fn predecessors(&self) -> Self::WallNodesList {
                self.successors()
            }
        }

        struct WallManagerType;

        impl WallChecker<WallType> for WallManagerType {
            type Error = core::convert::Infallible;

            fn try_exists(&self, wall: &WallType) -> Result<bool, Self::Error> {
                match wall.0 {
                    2 | 5 => Ok(true),
                    _ => Ok(false),
                }
            }

            fn try_is_checked(&self, wall: &WallType) -> Result<bool, Self::Error> {
                match wall.0 {
                    0 | 1 | 2 | 3 | 5 => Ok(true),
                    _ => Ok(false),
                }
            }
        }

        let manager = WallManagerType;

        let maze = CheckedMaze::<_, _, usize>::new(Rc::new(manager), ThroughPatternConverter);
        let expected = vec![(2usize, 2usize), (2, 1)]
            .into_iter()
            .map(|(node, cost)| (NodeType(node), cost))
            .collect::<Vec<(NodeType, usize)>>();
        assert_eq!(maze.successors(&NodeType(0)), expected.as_slice());
        assert_eq!(maze.predecessors(&NodeType(0)), expected.as_slice());
    }

    #[test]
    fn test_unchecked_node_finder() {
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

        let maze = Maze::<_, _, SearchNode>::new(Rc::new(manager), ThroughPatternConverter);

        let path = vec![0, 1, 1, 2, 3, 5, 8]
            .into_iter()
            .map(|e| Node(e))
            .collect::<Vec<_>>();
        let expected = vec![0, 1, 2, 3, 2, 3, 8, 9]
            .into_iter()
            .map(|e| SearchNode(e))
            .collect::<Vec<_>>();
        assert_eq!(maze.find_unchecked_nodes(&path), expected.as_slice());
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

        let maze = Maze::<_, _, usize>::new(Rc::new(manager), ThroughPatternConverter);
        let test_cases = vec![
            (0, Some(false)),
            (1, Some(true)),
            (2, None),
            (3, Some(false)),
        ];
        for (node, expected) in test_cases {
            assert_eq!(maze.is_available(&node), expected);
        }
    }
}
