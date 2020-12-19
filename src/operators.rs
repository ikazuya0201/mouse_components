use crate::administrator::NotFinishError;

pub trait Operator {
    type Mode;

    fn init(&self);
    fn tick(&self);
    fn run(&self) -> Result<Self::Mode, NotFinishError>;
}

pub mod search_operator;
