#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

///sample from normal distribution
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Sample<T> {
    pub mean: T,
    pub standard_deviation: T,
}
