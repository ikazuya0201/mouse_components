use heapless::{ArrayLength, Vec};

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
    fn set_direction_mapping(&self, node: Node, mapping: fn(fn(Direction) -> bool) -> Direction);
    fn instruct_direction(&self) -> Option<(Direction, Node)>;
}

pub trait DirectionalGraph<Node, Cost, Direction>: Graph<Node, Cost> {
    fn edge_direction(&self, edge: (Node, Node)) -> Direction;
}

pub trait CheckableGraph<Node, Cost>: Graph<Node, Cost> {
    type Nodes: Iterator<Item = Node>;

    fn is_checked(&self, edge: (Node, Node)) -> bool;
    fn unchecked_edge_to_target_nodes(&self, edge: (Node, Node)) -> Self::Nodes;
    fn related_target_nodes(&self, node: Node) -> Self::Nodes;
    fn checked_successors(&self, node: Node) -> Self::Edges;
    fn checked_predecessors(&self, node: Node) -> Self::Edges;
}

pub trait Graph<Node, Cost> {
    type Edges: Iterator<Item = (Node, Cost)>;

    fn successors(&self, node: Node) -> Self::Edges;
    fn predecessors(&self, node: Node) -> Self::Edges;
}
