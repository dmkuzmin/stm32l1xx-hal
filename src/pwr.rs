//! Power management

use crate::stm32::{pwr, PWR};

/// Extension trait that constrains the `PWR` peripheral
pub trait PwrExt {
    /// Constrains the `PWR` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Pwr;
}

impl PwrExt for PWR {
    fn constrain(self) -> Pwr {
        Pwr {
            csr: CSR { _0: () },
        }
    }
}

pub struct Pwr {
    /// Control/Status Register
    pub csr: CSR,
}

/// CSR Control/Status Register
pub struct CSR {
    _0: (),
}

impl CSR {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn csr(&mut self) -> &pwr::CSR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*PWR::ptr()).csr }
    }
}