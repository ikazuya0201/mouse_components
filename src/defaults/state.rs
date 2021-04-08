//! Definition of state.

use serde::{Deserialize, Serialize};

use crate::impl_with_getter;
use crate::nodes::Node;
use crate::tracker::RobotState;

impl_with_getter! {
    /// A state for operators.
    #[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
    pub struct State<const N: usize> {
        current_node: Node<N>,
        robot_state: RobotState,
    }
}

impl<const N: usize> State<N> {
    pub fn new(current_node: Node<N>, robot_state: RobotState) -> Self {
        Self {
            current_node,
            robot_state,
        }
    }
}
