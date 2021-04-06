//! Definition of state.

use serde::{Deserialize, Serialize};

use crate::impl_with_getter;
use crate::nodes::Node;
use crate::tracker::RobotState;

impl_with_getter! {
    /// A state for operators.
    #[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
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
