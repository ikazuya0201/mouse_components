use heapless::{ArrayLength, Vec};

use super::maze::DirectionalGraph;

pub trait Solver<Node, Cost, Direction, Graph, L>
where
    Graph: DirectionalGraph<Node, Cost, Direction>,
    L: ArrayLength<Node>,
{
    type Directions: IntoIterator<Item = Direction>;
    //return: (route, last direction candidates sequence)
    fn start_node(&self) -> Node;
    fn next_path(&self, current: Node, graph: &Graph) -> Option<Vec<Node, L>>;
    fn last_node(&self) -> Option<Node>;
    fn next_directions(&self, graph: &Graph) -> Option<Self::Directions>;
}
