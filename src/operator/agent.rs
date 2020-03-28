use heapless::{ArrayLength, Vec};

pub trait Agent<Position, Direction> {
    fn existing_obstacles<L: ArrayLength<Position>>(&self) -> Vec<Position, L>;
    fn position(&self) -> Position;
    fn set_next_route(&self, route: &[Position]);
    fn set_direction_mapping(&self, mapping: fn(fn(Direction) -> bool) -> Direction);
    fn is_route_empty(&self) -> bool;
    //This method is called by interrupt.
    fn track_next(&self);
}
