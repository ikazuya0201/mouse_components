mod atomic_enum;

use atomic_enum::AtomicEnum;

use Mode::*;

#[derive(PartialEq, Eq)]
pub enum Mode {
    Idle,    //do nothing
    Search,  //maze search
    FastRun, //fast run to goal and return to start
    Select,  //mode select
}

impl Mode {
    pub const fn size() -> u8 {
        4
    }
}

impl Into<u8> for Mode {
    fn into(self) -> u8 {
        match self {
            Idle => 0,
            Search => 1,
            FastRun => 2,
            Select => 3,
        }
    }
}

impl From<u8> for Mode {
    fn from(val: u8) -> Self {
        match val {
            0 => Idle,
            1 => Search,
            2 => FastRun,
            3 => Select,
            _ => unreachable!(),
        }
    }
}

pub type AtomicMode = AtomicEnum<Mode>;
