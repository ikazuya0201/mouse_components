pub trait Agent<Position, Pattern> {
    type Positions: IntoIterator<Item = Position>;

    fn existing_obstacles(&self) -> Self::Positions;
    fn set_next_path<Patterns: IntoIterator<Item = Pattern>>(&self, path: Patterns);
    fn set_instructed_pattern(&self, pattern: Pattern);
    fn is_route_empty(&self) -> bool;
    //This method is called by interrupt.
    fn track_next(&self);
}
