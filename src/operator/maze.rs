use heapless::{ArrayLength, Vec};

pub trait GraphTranslator<Node, Position> {
    fn node_to_position(&self, node: Node) -> Position;
    fn position_to_node(&self, position: Position) -> Node;
    fn update_obstacles(&self, positions: &[Position]);
}

pub trait Graph<Node, Cost, Direction> {
    fn neighbors<L: ArrayLength<(Node, Cost)>>(&self, node: Node) -> Vec<(Node, Cost), L>;
    fn is_edge_checked(&self, node1: Node, node2: Node) -> bool;
    fn edge_direction(&self, src: Node, dst: Node) -> Direction;
}
