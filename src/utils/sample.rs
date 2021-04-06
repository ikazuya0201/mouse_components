use serde::{Deserialize, Serialize};

///sample from normal distribution
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sample<T> {
    pub mean: T,
    pub standard_deviation: T,
}
