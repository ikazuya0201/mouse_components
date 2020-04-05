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

pub trait Graph<Node, Cost, Direction> {
    fn successors<L: ArrayLength<(Node, Cost)>>(&self, node: Node) -> Vec<(Node, Cost), L>;
    fn predecessors<L: ArrayLength<(Node, Cost)>>(&self, node: Node) -> Vec<(Node, Cost), L>;
    fn is_checked(&self, edge: (Node, Node)) -> bool;
    fn edge_direction(&self, edge: (Node, Node)) -> Direction;
    fn separate_to_unit_edges<L: ArrayLength<(Node, Node)>>(
        &self,
        edge: (Node, Node),
    ) -> Vec<(Node, Node), L>;
}
