use serde::{Deserialize, Serialize};

/// Random variable in normal distribution
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Serialize, Deserialize)]
pub struct Random<T> {
    pub mean: T,
    pub standard_deviation: T,
}
