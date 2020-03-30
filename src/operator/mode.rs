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

impl num::FromPrimitive for Mode {
    fn from_i64(n: i64) -> Option<Mode> {
        match n {
            0 => Some(Idle),
            1 => Some(Search),
            2 => Some(FastRun),
            3 => Some(Select),
            _ => None,
        }
    }

    fn from_u64(n: u64) -> Option<Mode> {
        Self::from_i64(n as i64)
    }
}

impl num::ToPrimitive for Mode {
    fn to_u64(&self) -> Option<u64> {
        match self {
            Idle => Some(0),
            Search => Some(1),
            FastRun => Some(2),
            Select => Some(3),
        }
    }

    fn to_i64(&self) -> Option<i64> {
        self.to_u64().map(|e| e as i64)
    }
}

pub type AtomicMode = AtomicEnum<Mode>;
