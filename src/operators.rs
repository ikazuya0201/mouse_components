//! Implementations of [Operator](crate::administrator::Operator) and their dependent traits.

mod return_setup_operator;
mod run_operator;
mod search_operator;

pub use return_setup_operator::{
    ReturnSetupAgent, ReturnSetupAgentError, ReturnSetupCommandConsumer, ReturnSetupCommander,
    ReturnSetupOperator,
};
pub use run_operator::{RunAgent, RunCommander, RunOperator};
pub use search_operator::{SearchAgent, SearchCommander, SearchCommanderError, SearchOperator};
