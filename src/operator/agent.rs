pub trait Agent<Position, Direction> {
    type Positions: IntoIterator<Item = Position>;

    fn existing_obstacles(&self) -> Self::Positions;
    fn set_instructed_direction(&self, direction: Direction);
    //This method is called by interrupt.
    fn track_next(&self);
}
