#![cfg_attr(not(test), no_std)]

extern crate alloc;

#[cfg(feature = "serde")]
pub mod logged_tracker;
pub mod math;
pub mod sensors;
