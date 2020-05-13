mod atomic_enum;

use atomic_enum::AtomicEnum;

#[derive(Debug, PartialEq, Eq)]
pub struct Idle;

#[derive(Debug, PartialEq, Eq)]
pub struct Search;

#[derive(Debug, PartialEq, Eq)]
pub struct FastRun;

#[derive(Debug, PartialEq, Eq)]
pub struct Select;

#[derive(Debug, PartialEq, Eq)]
pub enum Mode {
    Idle(Idle),       //do nothing
    Search(Search),   //maze search
    FastRun(FastRun), //fast run to goal and return to start
    Select(Select),   //mode select
}

impl Mode {
    pub const fn size() -> u8 {
        4
    }
}

impl Into<u8> for Mode {
    fn into(self) -> u8 {
        match self {
            Mode::Idle(Idle) => 0,
            Mode::Search(Search) => 1,
            Mode::FastRun(FastRun) => 2,
            Mode::Select(Select) => 3,
        }
    }
}

impl From<u8> for Mode {
    fn from(val: u8) -> Self {
        match val {
            0 => Mode::Idle(Idle),
            1 => Mode::Search(Search),
            2 => Mode::FastRun(FastRun),
            3 => Mode::Select(Select),
            _ => unreachable!(),
        }
    }
}

pub type AtomicMode = AtomicEnum<Mode>;
