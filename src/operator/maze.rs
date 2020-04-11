pub trait Storable {
    fn store(&self);
    fn restore(&self);
}

pub trait GraphTranslator<Node, Position> {
    fn node_to_position(&self, node: Node) -> Position;
    fn position_to_node(&self, position: Position) -> Node;
    fn update_obstacles(&self, positions: &[Position]);
}

pub trait DirectionInstructor<Node, Direction> {
    fn set_direction_iterator<Iter: IntoIterator<Item = Direction>>(&self, node: Node, iter: Iter);
    fn instruct_direction(&self) -> Option<(Direction, Node)>;
}

pub trait DirectionalGraph<Node, Cost, Direction>: CheckableGraph<Node, Cost> {
    fn edge_direction(&self, edge: (Node, Node)) -> Direction;
    //block outbound edge of the given node by the direction.
    //return inbound node of updated edges
    fn block(&mut self, node: Node, direction: Direction) -> Self::Nodes;
    fn find_first_checker_node_and_next_direction(&self, edge: (Node, Node)) -> (Node, Direction);
    fn unchecked_edge_direction(&self, node: Node) -> Option<Direction>;
}

pub trait CheckableGraph<Node, Cost>: Graph<Node, Cost> {
    type Nodes: IntoIterator<Item = Node>;

    fn is_checked(&self, edge: (Node, Node)) -> bool;
    fn unchecked_edge_to_checker_nodes(&self, edge: (Node, Node)) -> Self::Nodes;
    fn related_checker_nodes(&self, node: Node) -> Self::Nodes;
    fn checked_successors(&self, node: Node) -> Self::Edges;
    fn checked_predecessors(&self, node: Node) -> Self::Edges;
}

pub trait Graph<Node, Cost> {
    type Edges: IntoIterator<Item = (Node, Cost)>;

    fn successors(&self, node: Node) -> Self::Edges;
    fn predecessors(&self, node: Node) -> Self::Edges;
}
