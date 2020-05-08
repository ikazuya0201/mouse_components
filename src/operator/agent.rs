pub trait Agent<Position, Pattern> {
    type Positions: IntoIterator<Item = Position>;

    fn existing_obstacles(&self) -> Self::Positions;
    fn set_instructed_pattern(&self, pattern: Pattern);
    //This method is called by interrupt.
    fn track_next(&self);
}
