pub trait Storable {
    fn store(&self);
    fn restore(&self);
}

pub trait ObstacleInterpreter<Obstacle> {
    fn interpret_obstacles<Obstacles: IntoIterator<Item = Obstacle>>(&self, obstacles: Obstacles);
}

//The trait method could be called by other threads.
pub trait DirectionInstructor<Node, Direction> {
    fn update_node_candidates<Nodes: IntoIterator<Item = Node>>(&self, candidates: Nodes);
    fn instruct(&self, current: Node) -> Option<(Direction, Node)>;
}

pub trait GraphConverter<Node, SearchNode> {
    type SearchNodes: IntoIterator<Item = SearchNode>;

    ///checker nodes are search nodes on which agent can check
    ///if walls on the given path exist.
    fn convert_to_checker_nodes<Nodes: IntoIterator<Item = Node>>(
        &self,
        path: Nodes,
    ) -> Self::SearchNodes;
}

pub trait Graph<Node, Cost> {
    type Edges: IntoIterator<Item = (Node, Cost)>;

    fn successors(&self, node: Node) -> Self::Edges;
    fn predecessors(&self, node: Node) -> Self::Edges;
}

///convert node to pose
pub trait NodeConverter<Node, Pose> {
    fn convert(&self, node: Node) -> Pose;
}
