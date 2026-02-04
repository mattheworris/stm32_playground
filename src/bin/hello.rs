#![no_main]
#![no_std]

use aero_app as _; // global logger + panicking-behavior + memory layout

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Hello, world!");

    aero_app::exit()
}
