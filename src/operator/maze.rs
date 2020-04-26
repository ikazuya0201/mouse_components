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
    fn update_direction_candidates<Directions: IntoIterator<Item = Direction>>(
        &self,
        node: Node,
        directions: Directions,
    );
    fn instruct(&self) -> Option<(Pattern, Node)>;
}

pub trait DirectionalGraph<Node, Cost, Direction>: CheckableGraph<Node, Cost> {
    fn edge_direction(&self, edge: (Node, Node)) -> Direction;
    //block outbound edge of the given node by the direction.
    //return inbound node of updated edges
    fn block(&mut self, node: Node, direction: Direction) -> Self::Nodes;
    fn find_first_checker_node_and_next_direction(&self, edge: (Node, Node)) -> (Node, Direction);
}

pub trait CheckableGraph<Node, Cost>: Graph<Node, Cost> {
    type Nodes: IntoIterator<Item = Node>;

    fn is_checked(&self, edge: (Node, Node)) -> bool;
    fn unchecked_edge_to_checker_nodes(&self, edge: (Node, Node)) -> Self::Nodes;
    fn checked_successors(&self, node: Node) -> Self::Edges;
    fn checked_predecessors(&self, node: Node) -> Self::Edges;
    fn unchecked_successors(&self, node: Node) -> Self::Edges;
    fn unchecked_predecessors(&self, node: Node) -> Self::Edges;
}

pub trait Graph<Node, Cost> {
    type Edges: IntoIterator<Item = (Node, Cost)>;

    fn successors(&self, node: Node) -> Self::Edges;
    fn predecessors(&self, node: Node) -> Self::Edges;
}
