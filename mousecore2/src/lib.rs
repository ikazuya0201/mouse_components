#![cfg_attr(not(test), no_std)]

pub mod control;
pub mod estimator;
pub mod solver;
pub mod wall_detector;

const WIDTH: usize = 32;
