use super::maze::{Graph, GraphConverter};

pub trait Solver<Node, SearchNode, Cost, IGraph>
where
    IGraph: Graph<Node, Cost>,
{
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
        IGraph: Graph<SearchNode, Cost> + GraphConverter<Node, SearchNode>;
    fn compute_shortest_path(&self, graph: &IGraph) -> Option<Self::Nodes>;
}
