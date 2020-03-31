#![no_std]
#![no_main]

extern crate panic_rtt;

mod direction;
mod operator;
mod solver;

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    loop {}
}
