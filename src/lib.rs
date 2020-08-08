#![cfg_attr(not(test), no_std)]
#![feature(type_alias_impl_trait)]

pub mod administrator;
pub mod agent;
pub mod controller;
pub mod estimator;
pub mod maze;
pub mod obstacle_detector;
pub mod pattern;
pub mod prelude;
pub mod solver;
pub mod tracker;
pub mod trajectory_generator;
pub mod utils;

pub mod sensors {
    pub use super::estimator::{Encoder, IMU};
    pub use super::tracker::Motor;
}
