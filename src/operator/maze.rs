pub trait Storable {
    fn store(&self);
    fn restore(&self);
}

pub trait GraphTranslator<Node, Position, Pattern> {
    type Patterns: IntoIterator<Item = Pattern>;

    fn nodes_to_patterns<Nodes: IntoIterator<Item = Node>>(&self, nodes: Nodes) -> Self::Patterns;
    fn update_obstacles<Positions: IntoIterator<Item = Position>>(&self, states: Positions);
}

pub trait PatternInstructor<Node, Direction, Pattern> {
    fn update_node_candidates<Nodes: IntoIterator<Item = Node>>(
        &self,
        current: Node,
        candidates: Nodes,
    );
    fn instruct(&self) -> Option<(Pattern, Node)>;
}

pub trait CheckerGraph<Node> {
    type CheckerNodes: IntoIterator<Item = Node>;

    fn convert_to_checker_nodes<Nodes: IntoIterator<Item = Node>>(
        &self,
        path: Nodes,
    ) -> Self::CheckerNodes;
}

pub trait ReducedGraph<Node, Cost>: Graph<Node, Cost> {
    fn reduced_successors(&self, node: Node) -> Self::Edges;
    fn reduced_predecessors(&self, node: Node) -> Self::Edges;
}

pub trait Graph<Node, Cost> {
    type Edges: IntoIterator<Item = (Node, Cost)>;

    fn successors(&self, node: Node) -> Self::Edges;
    fn predecessors(&self, node: Node) -> Self::Edges;
}
