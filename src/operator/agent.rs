#[derive(PartialEq, Eq, Debug, Copy, Clone)]
pub enum AgentDirection {
    Forward,
    ForwardRight,
    ForwardLeft,
    Back,
    Right,
    Left,
}

pub trait Position {}

pub trait Agent<P: Position> {
    fn wall_exists(&self, dir: AgentDirection) -> bool;
    fn rotate(&mut self, dir: AgentDirection);
    fn position(&self) -> P;
    fn set_position_candidates(&self, positions: &[P]);
    //This method is called by interrupt.
    fn track(&self);
}
