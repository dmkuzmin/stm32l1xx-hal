//! Delays

use crate::rcc::Clocks;
use cast::u32;
use cortex_m::peripheral::DCB;
use cortex_m::peripheral::DWT;
use hal::blocking::delay::{DelayMs, DelayUs};

/// Serial error
#[derive(Debug)]
pub enum Error {
    /// Overflow error
    Overflow,
}

/// Data Watchpoint and Trace unit timer (DWT) as a delay provider
pub struct Delay {
    dcb: DCB,
    dwt: DWT,
    clocks: Clocks,
}

impl Delay {
    /// Data Watchpoint and Trace unit timer (DWT) as a delay provider
    pub fn new(mut dcb: DCB, mut dwt: DWT, clocks: Clocks) -> Self {
        dcb.enable_trace();
        dwt.enable_cycle_counter();

        Delay { dcb, dwt, clocks }
    }

    /// Debug Control Block (DCB) and Data Watchpoint and Trace unit timer (DWT) resource
    pub fn free(self) -> (DCB, DWT) {
        (self.dcb, self.dwt)
    }
}

impl DelayMs<u32> for Delay {
    type Error = Error;

    fn try_delay_ms(&mut self, ms: u32) -> Result<Self, Error> {
        self.try_delay_us(ms * 1_000)
    }
}

impl DelayMs<u16> for Delay {
    type Error = Error;

    fn try_delay_ms(&mut self, ms: u16) -> Result<Self, Error> {
        self.try_delay_ms(u32(ms))
    }
}

impl DelayMs<u8> for Delay {
    type Error = Error;

    fn try_delay_ms(&mut self, ms: u8) -> Result<Self, Error> {
        self.try_delay_ms(u32(ms))
    }
}

impl DelayUs<u32> for Delay {
    type Error = Error;

    fn try_delay_us(&mut self, us: u32) -> Result<Self, Error> {
        let t0 = self.dwt.get_cycle_count();
        let rvr = match us.checked_mul(self.clocks.sysclk().0 / 1_000_000) {
            Some(rvr) => rvr,
            None => return Err(nb::Error::Other(Error::Parity)),
        };
        while self.dwt.get_cycle_count().wrapping_sub(t0) < rvr {}
        Ok(Self)
    }
}

impl DelayUs<u16> for Delay {
    type Error = Error;

    fn try_delay_us(&mut self, us: u16) -> Result<Self, Error> {
        self.try_delay_us(u32(us))
    }
}

impl DelayUs<u8> for Delay {
    type Error = Error;

    fn try_delay_us(&mut self, us: u8) -> Result<Self, Error> {
        self.try_delay_us(u32(us))
    }
}
