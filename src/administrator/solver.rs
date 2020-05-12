use super::maze::{Graph, GraphConverter};

pub trait Solver<Node, SearchNode, Cost, IGraph> {
    type Nodes: IntoIterator<Item = Node>;
    type SearchNodes: IntoIterator<Item = SearchNode>;
    //return: (route, last direction candidates sequence)
    fn start_search_node(&self) -> SearchNode;
    fn next_node_candidates(
        &self,
        current: SearchNode,
        graph: &IGraph,
    ) -> Option<Self::SearchNodes>
    where
        IGraph: Graph<SearchNode, Cost> + GraphConverter<Node, SearchNode> + Graph<Node, Cost>;
    fn compute_shortest_path(&self, graph: &IGraph) -> Option<Self::Nodes>
    where
        IGraph: Graph<Node, Cost>;
}
