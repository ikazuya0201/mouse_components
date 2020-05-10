pub trait Storable {
    fn store(&self);
    fn restore(&self);
}

pub trait ObstacleInterpreter<Position> {
    fn interpret_obstacles<Positions: IntoIterator<Item = Position>>(&self, positions: Positions);
}

pub trait DirectionInstructor<Node, Direction> {
    fn update_node_candidates<Nodes: IntoIterator<Item = Node>>(
        &self,
        current: Node,
        candidates: Nodes,
    );
    fn instruct(&self) -> Option<(Direction, Node)>;
}

pub trait GraphConverter<Node, SearchNode> {
    type SearchNodes: IntoIterator<Item = SearchNode>;

    fn convert_to_checker_nodes<Nodes: IntoIterator<Item = Node>>(
        &self,
        path: Nodes,
    ) -> Self::SearchNodes;
}

pub trait Graph<Node, Cost> {
    type Edges: IntoIterator<Item = (Node, Cost)>;

    fn successors(&self, node: Node) -> Self::Edges;
    fn predecessors(&self, node: Node) -> Self::Edges;
}
