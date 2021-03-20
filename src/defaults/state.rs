use crate::impl_with_getter;
use crate::tracker::RobotState;

impl_with_getter! {
    pub struct State<Node> {
        current_node: Node,
        robot_state: RobotState,
    }
}

impl<Node> State<Node> {
    pub fn new(current_node: Node, robot_state: RobotState) -> Self {
        Self {
            current_node,
            robot_state,
        }
    }
}
