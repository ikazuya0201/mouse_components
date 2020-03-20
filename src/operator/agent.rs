#[derive(PartialEq, Eq, Debug, Copy, Clone)]
pub enum AgentDirection {
    Forward,
    Back,
    Right,
    Left,
}

use super::trajectory_generator::Target;

pub trait Position {}

pub trait Agent<P: Position, T: Target> {
    fn wall_exists(&mut self, dir: AgentDirection) -> bool;
    fn rotate(&mut self, dir: AgentDirection);
    fn position(&self) -> P;
    fn set_trajectory(&mut self, trajectory: &[T]);
    //This method is called by interrupt.
    fn track(&mut self);
}
