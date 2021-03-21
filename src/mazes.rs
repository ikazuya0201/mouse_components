//! Definition of [Maze](crate::mazes::Maze) and its dependent traits.

use core::fmt;
use core::marker::PhantomData;

use heapless::{ArrayLength, Vec};

use crate::commanders::{BoundedNode, Graph, GraphConverter, NodeChecker};
use crate::utils::forced_vec::ForcedVec;

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

/// A trait that enumerates wall-node successors and predecessors of a node.
pub trait GraphNode: Sized + WallSpaceNode {
    type NeighborNum;
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

/// An implementation of [Graph](crate::commanders::Graph),
/// [GraphConverter](crate::commanders::GraphConverter) and
/// [NodeChecker](crate::commanders::NodeChecker) required by
/// [SearchCommander](crate::commanders::SearchCommander).
pub struct Maze<'a, Manager, Pattern, Cost, SearchNode> {
    manager: &'a Manager,
    cost_fn: fn(Pattern) -> Cost,
    _search_node: PhantomData<fn() -> SearchNode>,
}

impl<'a, Manager, Pattern, Cost, SearchNode> fmt::Debug
    for Maze<'a, Manager, Pattern, Cost, SearchNode>
where
    Manager: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Maze")
            .field("manager", &self.manager)
            .finish()
    }
}

impl<'a, Manager, Pattern, Cost, SearchNode> fmt::Display
    for Maze<'a, Manager, Pattern, Cost, SearchNode>
where
    Manager: fmt::Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.manager.fmt(f)
    }
}

impl<'a, Manager, Pattern, Cost, SearchNode> Maze<'a, Manager, Pattern, Cost, SearchNode> {
    pub fn new(manager: &'a Manager, cost_fn: fn(Pattern) -> Cost) -> Self {
        Self {
            manager,
            cost_fn,
            _search_node: PhantomData,
        }
    }
}

impl<'a, Manager, Pattern, Cost, SearchNode> Maze<'a, Manager, Pattern, Cost, SearchNode> {
    fn _successors<Node, F>(&self, node: &Node, is_blocked: F) -> <Self as Graph<Node>>::Edges
    where
        F: Fn(&Node::Wall) -> bool,
        Node: GraphNode<Pattern = Pattern>,
        Manager: WallChecker<Node::Wall>,
        Node::NeighborNum: ArrayLength<(Node, Cost)>,
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
                        successors.push((node, (self.cost_fn)(pattern)));
                    }
                }
            }
        }
        successors.into()
    }

    fn _predecessors<Node, F>(&self, node: &Node, is_blocked: F) -> <Self as Graph<Node>>::Edges
    where
        F: Fn(&Node::Wall) -> bool,
        Node: GraphNode<Pattern = Pattern>,
        Manager: WallChecker<Node::Wall>,
        Node::NeighborNum: ArrayLength<(Node, Cost)>,
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
                        predecessors.push((node, (self.cost_fn)(pattern)));
                    }
                }
            }
        }
        predecessors.into()
    }
}

impl<'a, Node, Manager, Cost, SearchNode> Graph<Node>
    for Maze<'a, Manager, Node::Pattern, Cost, SearchNode>
where
    Node: GraphNode,
    Manager: WallChecker<Node::Wall>,
    Node::NeighborNum: ArrayLength<(Node, Cost)>,
{
    type Cost = Cost;
    type Edges = Vec<(Node, Self::Cost), Node::NeighborNum>;

    fn successors(&self, node: &Node) -> Self::Edges {
        //Define blocked state as existence of wall (doesn't consider whether a wall is checked or not).
        self._successors(node, |wall| self.manager.exists(wall))
    }

    fn predecessors(&self, node: &Node) -> Self::Edges {
        //Define blocked state as existence of wall (doesn't consider whether a wall is checked or not).
        self._predecessors(node, |wall| self.manager.exists(wall))
    }
}

impl<'a, Node, Manager, Pattern, Cost, SearchNode> GraphConverter<Node>
    for Maze<'a, Manager, Pattern, Cost, SearchNode>
where
    Node: WallFinderNode,
    Manager: WallChecker<Node::Wall>,
    Node::Wall: Into<[SearchNode; 2]>,
    SearchNode: BoundedNode,
    SearchNode::UpperBound: ArrayLength<SearchNode>,
{
    type SearchNode = SearchNode;
    type SearchNodes = Vec<Self::SearchNode, SearchNode::UpperBound>;

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
                let [node1, node2] = wall.into();
                nodes.push(node1);
                nodes.push(node2);
            }
        }
        nodes.into()
    }
}

impl<'a, Manager, Pattern, Cost, SearchNode> NodeChecker<SearchNode>
    for Maze<'a, Manager, Pattern, Cost, SearchNode>
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
pub struct CheckedMaze<'a, Manager, Pattern, Cost, SearchNode>(
    Maze<'a, Manager, Pattern, Cost, SearchNode>,
);

impl<'a, Manager, Pattern, Cost, SearchNode> CheckedMaze<'a, Manager, Pattern, Cost, SearchNode> {
    pub fn new(manager: &'a Manager, cost_fn: fn(Pattern) -> Cost) -> Self {
        Self(<Self as core::ops::Deref>::Target::new(manager, cost_fn))
    }
}

impl<'a, Manager, Pattern, Cost, SearchNode> core::ops::Deref
    for CheckedMaze<'a, Manager, Pattern, Cost, SearchNode>
{
    type Target = Maze<'a, Manager, Pattern, Cost, SearchNode>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<'a, Node, Manager, Pattern, Cost, SearchNode> Graph<Node>
    for CheckedMaze<'a, Manager, Pattern, Cost, SearchNode>
where
    Node: GraphNode<Pattern = Pattern>,
    Manager: WallChecker<Node::Wall>,
    Node::NeighborNum: ArrayLength<(Node, Cost)>,
{
    type Cost = Cost;
    type Edges = Vec<(Node, Self::Cost), Node::NeighborNum>;

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

        let cost = |pattern: usize| -> usize { pattern };

        let maze = Maze::<_, _, _, usize>::new(&manager, cost);
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
            type NeighborNum = U10;
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

        let cost = |pattern: usize| -> usize { pattern };

        let maze = CheckedMaze::<_, _, _, usize>::new(&manager, cost);
        let expected = vec![(2usize, 2usize), (2, 1)]
            .into_iter()
            .map(|(node, cost)| (NodeType(node), cost))
            .collect::<Vec<(NodeType, usize)>>();
        assert_eq!(maze.successors(&NodeType(0)), expected.as_slice());
        assert_eq!(maze.predecessors(&NodeType(0)), expected.as_slice());
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

        let cost = |_pattern: usize| -> usize { unreachable!() };

        let maze = Maze::<_, _, _, SearchNode>::new(&manager, cost);

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

        let cost = |_pattern: usize| -> usize { unreachable!() };

        let maze = Maze::<_, _, _, usize>::new(&manager, cost);
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
