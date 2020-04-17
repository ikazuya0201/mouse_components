use super::maze::DirectionalGraph;

pub trait Solver<Node, Cost, Direction, Graph>
where
    Graph: DirectionalGraph<Node, Cost, Direction>,
{
    type Nodes: IntoIterator<Item = Node>;
    type Directions: IntoIterator<Item = Direction>;
    //return: (route, last direction candidates sequence)
    fn start_node(&self) -> Node;
    fn update_node(&self, node: Node, graph: &Graph);
    fn next_path(&self, current: Node, graph: &Graph) -> Option<Self::Nodes>;
    fn last_node(&self) -> Option<Node>;
    fn next_direction_candidates(&self, graph: &Graph) -> Option<Self::Directions>;
}
