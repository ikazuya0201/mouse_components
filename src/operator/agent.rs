use super::direction::RelativeDirection;

pub trait Position {}

pub trait Agent<P: Position, D: RelativeDirection> {
    fn wall_exists(&self, dir: D) -> bool;
    fn position(&self) -> P;
    fn set_next_route(&self, route: &[P]);
    fn is_route_empty(&self) -> bool;
    //This method is called by interrupt.
    fn track_next(&self);
}
