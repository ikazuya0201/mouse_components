use crate::impl_with_getter;
use crate::nodes::Node;
use crate::tracker::RobotState;

impl_with_getter! {
    pub struct State<Size> {
        current_node: Node<Size>,
        robot_state: RobotState,
    }
}

impl<Size> State<Size> {
    pub fn new(current_node: Node<Size>, robot_state: RobotState) -> Self {
        Self {
            current_node,
            robot_state,
        }
    }
}
