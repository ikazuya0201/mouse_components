#![cfg_attr(not(test), no_std)]

pub mod control;
pub mod solver;
pub mod state;
pub mod trajectory;
pub mod wall;

const WIDTH: usize = 32;
