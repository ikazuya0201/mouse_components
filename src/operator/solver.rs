use super::maze;

pub trait Solver<Node, Direction> {
    type Directions: IntoIterator<Item = Direction>;
    //return: (route, last direction candidates sequence)
    fn start_node(&self) -> Node;
    fn next_direction_candidates<Cost, Graph>(
        &self,
        current: Node,
        graph: &Graph,
    ) -> Option<Self::Directions>
    where
        Graph: maze::Graph<Node, Cost>;
}
