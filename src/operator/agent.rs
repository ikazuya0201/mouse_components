pub trait Agent<Position, Direction> {
    type Positions: IntoIterator<Item = Position>;

    fn existing_obstacles(&self) -> Self::Positions;
    fn position(&self) -> Position;
    fn set_next_path<Positions: IntoIterator<Item = Position>>(&self, path: Positions);
    fn set_instructed_direction(&self, dir: Direction);
    fn is_route_empty(&self) -> bool;
    //This method is called by interrupt.
    fn track_next(&self);
}
