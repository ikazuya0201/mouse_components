//! Definition of state.

use serde::{Deserialize, Serialize};

use crate::nodes::{Node, NodeCreationError, RunNode};
use crate::tracker::RobotState;
use crate::types::states::*;
use crate::utils::builder::RequiredFieldEmptyError;
use crate::{get_or_err, impl_setter, impl_with_getter, Merge};

impl_with_getter! {
    /// A state for operators.
    #[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
    pub struct State<const N: usize> {
        current_node: RunNode<N>,
        robot_state: RobotState,
    }
}

impl<const N: usize> State<N> {
    pub fn new(current_node: RunNode<N>, robot_state: RobotState) -> Self {
        Self {
            current_node,
            robot_state,
        }
    }
}

impl<const N: usize> From<State<N>> for StateContainer<N> {
    fn from(value: State<N>) -> Self {
        StateContainer {
            node_commander: CommanderState {
                current_node: value.current_node.clone().into(),
            },
            run_node_commander: CommanderState {
                current_node: value.current_node,
            },
            estimator: EstimatorState {
                robot_state: value.robot_state,
            },
        }
    }
}

/// A helper type for [State]. This implements `AsRef` for construction of types.
pub struct StateContainer<const N: usize> {
    node_commander: CommanderState<Node<N>>,
    run_node_commander: CommanderState<RunNode<N>>,
    estimator: EstimatorState,
}

impl<const N: usize> AsRef<CommanderState<RunNode<N>>> for StateContainer<N> {
    fn as_ref(&self) -> &CommanderState<RunNode<N>> {
        &self.run_node_commander
    }
}

impl<const N: usize> AsRef<CommanderState<Node<N>>> for StateContainer<N> {
    fn as_ref(&self) -> &CommanderState<Node<N>> {
        &self.node_commander
    }
}

impl<const N: usize> AsRef<EstimatorState> for StateContainer<N> {
    fn as_ref(&self) -> &EstimatorState {
        &self.estimator
    }
}

/// Builder for [State].
pub struct StateBuilder<const N: usize> {
    current_node: Option<Node<N>>,
    robot_state: Option<RobotState>,
}

/// Error for [StateBuilder].
#[derive(Clone, PartialEq, Eq, Debug)]
pub enum StateBuilderError {
    FieldEmpty(RequiredFieldEmptyError),
    ConversionFailed(NodeCreationError),
}

impl From<RequiredFieldEmptyError> for StateBuilderError {
    fn from(value: RequiredFieldEmptyError) -> Self {
        Self::FieldEmpty(value)
    }
}

impl From<NodeCreationError> for StateBuilderError {
    fn from(value: NodeCreationError) -> Self {
        Self::ConversionFailed(value)
    }
}

impl<const N: usize> StateBuilder<N> {
    pub fn new() -> Self {
        Self {
            current_node: None,
            robot_state: None,
        }
    }

    impl_setter!(current_node: Node<N>);
    impl_setter!(robot_state: RobotState);

    pub fn build(&mut self) -> Result<State<N>, StateBuilderError> {
        use core::convert::TryInto;

        Ok(State {
            current_node: get_or_err!(self.current_node).try_into()?,
            robot_state: get_or_err!(self.robot_state),
        })
    }
}

impl<const N: usize> Default for StateBuilder<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> From<CommanderState<Node<N>>> for StateBuilder<N> {
    fn from(value: CommanderState<Node<N>>) -> Self {
        Self {
            current_node: Some(value.current_node),
            robot_state: None,
        }
    }
}

impl<const N: usize> From<CommanderState<RunNode<N>>> for StateBuilder<N> {
    fn from(value: CommanderState<RunNode<N>>) -> Self {
        Self {
            current_node: Some(value.current_node.into()),
            robot_state: None,
        }
    }
}

impl<const N: usize> From<EstimatorState> for StateBuilder<N> {
    fn from(value: EstimatorState) -> Self {
        Self {
            current_node: None,
            robot_state: Some(value.robot_state),
        }
    }
}

impl<const N: usize> Merge for StateBuilder<N> {
    fn merge(mut self, mut other: Self) -> Self {
        self.current_node = self.current_node.or_else(|| other.current_node.take());
        self.robot_state = self.robot_state.or_else(|| other.robot_state.take());
        self
    }
}
