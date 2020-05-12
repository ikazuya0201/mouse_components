use super::Mode;

pub trait Operator {
    fn tick(&self);
    fn run(&self) -> Result<(), Mode>;
}

mod search_operator;

pub use search_operator::SearchOperator;
