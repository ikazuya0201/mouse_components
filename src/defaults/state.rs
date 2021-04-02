//! Definition of state.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::impl_with_getter;
use crate::nodes::Node;
use crate::tracker::RobotState;

impl_with_getter! {
    /// A state for operators.
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    #[derive(Clone, PartialEq, Debug)]
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
