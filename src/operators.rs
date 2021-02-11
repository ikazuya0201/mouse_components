//! Implementations of [Operator](crate::administrator::Operator) and their dependent traits.

mod search_operator;
mod tracking_operator;

pub use search_operator::{SearchAgent, SearchCommander, SearchCommanderError, SearchOperator};
pub use tracking_operator::{
    InitialCommander, TrackingAgent, TrackingAgentError, TrackingInitializer, TrackingOperator,
};
