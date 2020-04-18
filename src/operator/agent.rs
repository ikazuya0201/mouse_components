pub trait Agent<AgentState, Direction> {
    type AgentStates: IntoIterator<Item = AgentState>;

    fn existing_obstacles(&self) -> Self::AgentStates;
    fn position(&self) -> AgentState;
    fn set_next_path<AgentStates: IntoIterator<Item = AgentState>>(&self, path: AgentStates);
    fn set_instructed_direction(&self, dir: Direction);
    fn is_route_empty(&self) -> bool;
    //This method is called by interrupt.
    fn track_next(&self);
}
