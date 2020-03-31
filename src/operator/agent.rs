use heapless::{ArrayLength, Vec};

pub trait Agent<Position, Direction> {
    fn existing_obstacles<L: ArrayLength<Position>>(&self) -> Vec<Position, L>;
    fn position(&self) -> Position;
    fn set_next_route(&self, route: &[Position]);
    fn set_instructed_direction(&self, dir: Direction);
    fn is_route_empty(&self) -> bool;
    //This method is called by interrupt.
    fn track_next(&self);
}
