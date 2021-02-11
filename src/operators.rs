//! Implementations of [Operator](crate::administrator::Operator) and their dependent traits.

mod return_setup_operator;
mod search_operator;
mod tracking_operator;

pub use return_setup_operator::{
    ReturnSetupAgent, ReturnSetupAgentError, ReturnSetupCommandConsumer, ReturnSetupCommander,
    ReturnSetupOperator,
};
pub use search_operator::{SearchAgent, SearchCommander, SearchCommanderError, SearchOperator};
pub use tracking_operator::{
    InitialCommander, TrackingAgent, TrackingAgentError, TrackingInitializer, TrackingOperator,
};
