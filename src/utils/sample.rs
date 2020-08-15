///sample from normal distribution
#[derive(Debug, Clone, PartialEq)]
pub struct Sample<T> {
    pub mean: T,
    pub standard_deviation: T,
}
