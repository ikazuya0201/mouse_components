#![cfg_attr(not(test), no_std)]

pub mod control;
pub mod estimate;
pub mod solve;
pub mod trajectory;
pub mod wall;

const WIDTH: usize = 32;
