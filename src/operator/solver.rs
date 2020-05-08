use super::maze::{CheckerGraph, ReducedGraph};

pub trait Solver<Node, Cost> {
    type Nodes: IntoIterator<Item = Node>;
    //return: (route, last direction candidates sequence)
    fn start_node(&self) -> Node;
    fn next_node_candidates<Graph>(&self, current: Node, graph: &Graph) -> Option<Self::Nodes>
    where
        Graph: ReducedGraph<Node, Cost> + CheckerGraph<Node>;
}
