pub trait Agent<Obstacle, Pose, Direction> {
    type Obstacles: IntoIterator<Item = Obstacle>;

    fn get_existing_obstacles(&self) -> Self::Obstacles;
    fn set_instructed_direction(&self, pose: Pose, direction: Direction);
    //This method is called by interrupt.
    fn track_next(&self);
}
