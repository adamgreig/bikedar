#![feature(used)]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate stm32;

use core::u16;
use cortex_m::asm;
use stm32::f7x7::{GPIOB, RCC};

#[inline(never)]
fn main() {
    cortex_m::interrupt::free(|cs| {
        let gpiob = GPIOB.borrow(cs);
        let rcc = RCC.borrow(cs);
        rcc.ahb1enr.modify(|_, w| w.gpioben().bit(true));
        gpiob.moder.modify(|_, w| w.moder7().output());
        gpiob.bsrr.write(|w| w.bs7().set());
    });
}

#[allow(dead_code)]
#[used]
#[link_section = ".rodata.interrupts"]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    asm::bkpt();
}
