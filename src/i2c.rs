//! Inter-Integrated Circuit (I2C) bus

use crate::stm32::{I2C1, I2C2};
use cast::u8;

use crate::gpio::gpiob::{PB10, PB11, PB6, PB7, PB8, PB9};
use crate::gpio::{Alternate, OpenDrain, Output, AF4};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::rcc::{Clocks, APB1};
use crate::time::Hertz;

/// I2C error
#[derive(Debug)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// NACK
    Nack,
    // Overrun, // slave mode only
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
    #[doc(hidden)]
    _Extensible,
}

// FIXME these should be "closed" traits
/// SCL pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SclPin<I2C> {}

/// SDA pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SdaPin<I2C> {}

unsafe impl SclPin<I2C1> for PB6<Alternate<AF4, Output<OpenDrain>>> {}
unsafe impl SclPin<I2C1> for PB8<Alternate<AF4, Output<OpenDrain>>> {}

unsafe impl SclPin<I2C2> for PB10<Alternate<AF4, Output<OpenDrain>>> {}

unsafe impl SdaPin<I2C1> for PB7<Alternate<AF4, Output<OpenDrain>>> {}
unsafe impl SdaPin<I2C1> for PB9<Alternate<AF4, Output<OpenDrain>>> {}

unsafe impl SdaPin<I2C2> for PB11<Alternate<AF4, Output<OpenDrain>>> {}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident) => {
        loop {
            let sr1 = $i2c.sr1.read();

            if sr1.berr().bit_is_set() {
                return Err(Error::Bus);
            } else if sr1.arlo().bit_is_set() {
                return Err(Error::Arbitration);
            } else if sr1.af().bit_is_set() {
                return Err(Error::Nack);
            } else if sr1.$flag().bit_is_set() {
                break;
            } else {
                // try again
            }
        }
    };
}

macro_rules! hal {
    ($($I2CX:ident: ($i2cX:ident, $i2cXen:ident, $i2cXrst:ident),)+) => {
        $(
            impl<SCL, SDA> I2c<$I2CX, (SCL, SDA)> {
                /// Configures the I2C peripheral to work in master mode
                pub fn $i2cX<F>(
                    i2c: $I2CX,
                    pins: (SCL, SDA),
                    speed: F,
                    clocks: Clocks,
                    apb1: &mut APB1,
                ) -> Self where
                    F: Into<Hertz>,
                    SCL: SclPin<$I2CX>,
                    SDA: SdaPin<$I2CX>,
                {
                    let speed = speed.into().0;
                    assert!(speed <= 400_000);

                    // Enable clock for I2C
                    apb1.enr().modify(|_, w| w.$i2cXen().set_bit());

                    // Reset I2C
                    apb1.rstr().modify(|_, w| w.$i2cXrst().set_bit());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().clear_bit());

                    // Make sure the I2C unit is disabled so we can configure it
                    i2c.cr1.modify(|_, w| w.pe().clear_bit());

                    // Calculate settings for I2C speed modes
                    let clock = clocks.pclk1().0;
                    let freq = clock / 1_000_000;
                    assert!(freq >= 2 && freq <= 50);

                    // Configure bus frequency into I2C peripheral
                    i2c.cr2.write(|w| unsafe { w.freq().bits(freq as u8) });

                    let trise = if speed <= 100_000 {
                        freq + 1
                    } else {
                        (freq * 300) / 1000 + 1
                    };

                    // Configure correct rise times
                    i2c.trise.write(|w| w.trise().bits(trise as u8));

                    // I2C clock control calculation
                    if speed <= 100_000 {
                        let ccr = {
                            let ccr = clock / (speed * 2);
                            if ccr < 4 {
                                4
                            } else {
                                ccr
                            }
                        };

                        // Set clock to standard mode with appropriate parameters for selected speed
                        i2c.ccr.write(|w| unsafe {
                            w.f_s()
                                .clear_bit()
                                .duty()
                                .clear_bit()
                                .ccr()
                                .bits(ccr as u16)
                        });
                    } else {
                        const DUTYCYCLE: u8 = 0;
                        if DUTYCYCLE == 0 {
                            let ccr = clock / (speed * 3);
                            let ccr = if ccr < 1 { 1 } else { ccr };

                            // Set clock to fast mode with appropriate parameters for selected speed (2:1 duty cycle)
                            i2c.ccr.write(|w| unsafe {
                                w.f_s().set_bit().duty().clear_bit().ccr().bits(ccr as u16)
                            });
                        } else {
                            let ccr = clock / (speed * 25);
                            let ccr = if ccr < 1 { 1 } else { ccr };

                            // Set clock to fast mode with appropriate parameters for selected speed (16:9 duty cycle)
                            i2c.ccr.write(|w| unsafe {
                                w.f_s().set_bit().duty().set_bit().ccr().bits(ccr as u16)
                            });
                        }
                    }

                    // Enable the I2C processing
                    i2c.cr1.modify(|_, w| w.pe().set_bit());

                    I2c { i2c, pins }
                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, (SCL, SDA)) {
                    (self.i2c, self.pins)
                }
            }

            impl<PINS> Read for I2c<$I2CX, PINS> {
                type Error = Error;

                fn try_read(
                    &mut self,
                    address: u8,
                    buffer: &mut [u8]
                ) -> Result<(), Self::Error>
                {
                    // Also wait until signalled we're master and everything is waiting for us
                    while {
                        let sr2 = self.i2c.sr2.read();
                        sr2.busy().bit_is_set()
                    } {}

                    // clear POS and set ACK, START bits
                    self.i2c.cr1.modify(|_, w| unsafe {
                        w.pos()
                            .clear_bit()
                            .ack()
                            .set_bit()
                            .start()
                            .set_bit()
                    });

                    // Wait until START condition was generated
                    busy_wait!(self.i2c, sb);

                    // Set up current address, we're trying to talk to
                    self.i2c.dr.write(|w| unsafe { w.bits((u32::from(address) << 1) + 1) });

                    // Wait until address was sent
                    busy_wait!(self.i2c, addr);

                    if buffer.len() == 1 {
                        // clear ACK bit
                        self.i2c.cr1.modify(|_, w| w.ack().clear_bit());
                        // Clear condition by reading SR2
                        self.i2c.sr2.read();
                        // Send STOP condition
                        self.i2c.cr1.modify(|_, w| w.stop().set_bit());
                    } else if buffer.len() == 2 {
                        // clear ACK bit
                        self.i2c.cr1.modify(|_, w| w.ack().clear_bit());
                        // POS
                        self.i2c.cr1.modify(|_, w| w.pos().set_bit());
                        // Clear condition by reading SR2
                        self.i2c.sr2.read();
                    } else {
                        // set ACK bit
                        self.i2c.cr1.modify(|_, w| w.ack().set_bit());
                        // Clear condition by reading SR2
                        self.i2c.sr2.read();
                    }

                    let mut iter = IntoIterator::into_iter(buffer);
                    loop {
                        match iter.next() {
                            Some(byte) => {
                                if iter.size_hint().0 <= 2 {
                                    if iter.size_hint().0 == 0 {
                                        // Wait until we have received something
                                        busy_wait!(self.i2c, rx_ne);
                                        *byte = self.i2c.dr.read().bits() as u8;
                                    } else if iter.size_hint().0 == 1 {
                                        // Wait until BTF
                                        busy_wait!(self.i2c, btf);
                                        // Send STOP condition
                                        self.i2c.cr1.modify(|_, w| w.stop().set_bit());
                                        *byte = self.i2c.dr.read().bits() as u8;
                                        match iter.next() {
                                            Some(byte) => *byte = self.i2c.dr.read().bits() as u8,
                                            None => break,
                                        }
                                        break;
                                    } else {
                                        // Wait until BTF
                                        busy_wait!(self.i2c, btf);
                                        // Send a START condition and set ACK bit
                                        self.i2c.cr1.modify(|_, w| w.ack().clear_bit());
                                        *byte = self.i2c.dr.read().bits() as u8;
                                        // Wait until BTF
                                        busy_wait!(self.i2c, btf);
                                        // Send STOP condition
                                        self.i2c.cr1.modify(|_, w| w.stop().set_bit());
                                        match iter.next() {
                                            Some(byte) => *byte = self.i2c.dr.read().bits() as u8,
                                            None => break,
                                        }
                                        match iter.next() {
                                            Some(byte) => *byte = self.i2c.dr.read().bits() as u8,
                                            None => break,
                                        }
                                        break;
                                    }
                                } else {
                                    // Wait until we have received something
                                    busy_wait!(self.i2c, rx_ne);
                                    *byte = self.i2c.dr.read().bits() as u8;
                                    if self.i2c.sr1.read().btf().bit_is_set() {
                                        match iter.next() {
                                            Some(byte) => *byte = self.i2c.dr.read().bits() as u8,
                                            None => break,
                                        }
                                    }
                                }
                            },
                            None => break,
                        }
                    }
                    /*for byte in buffer {
                        if buffer.len() <= 3 {
                            if buffer.len() == 1 {
                                // Wait until we have received something
                                busy_wait!(self.i2c, rx_ne);
                                *byte = self.i2c.dr.read().bits() as u8;
                            } else if buffer.len() == 2 {
                                // Wait until BTF
                                busy_wait!(self.i2c, btf);
                                // Send STOP condition
                                self.i2c.cr1.modify(|_, w| w.stop().set_bit());
                                *byte = self.i2c.dr.read().bits() as u8;
                                byte = byte + 1;
                                *byte = self.i2c.dr.read().bits() as u8;
                                break;
                            } else {
                                // Wait until BTF
                                busy_wait!(self.i2c, btf);
                                // Send a START condition and set ACK bit
                                self.i2c.cr1.modify(|_, w| w.ack().clear_bit());
                                *byte = self.i2c.dr.read().bits() as u8;
                                // Wait until BTF
                                busy_wait!(self.i2c, btf);
                                // Send STOP condition
                                self.i2c.cr1.modify(|_, w| w.stop().set_bit());
                                *byte = self.i2c.dr.read().bits() as u8;
                                byte = byte + 1;
                                *byte = self.i2c.dr.read().bits() as u8;
                                break;
                            }
                        } else {
                            // Wait until we have received something
                            busy_wait!(self.i2c, rx_ne);
                            *byte = self.i2c.dr.read().bits() as u8;
                            if self.i2c.sr1.btf.read().is_set_bit()
                            {
                                byte = byte + 1;
                                *byte = self.i2c.dr.read().bits() as u8;
                            }
                        }
                    }*/

                    // Fallthrough is success
                    Ok(())
                }
            }

            impl<PINS> Write for I2c<$I2CX, PINS> {
                type Error = Error;

                fn try_write(
                    &mut self,
                    address: u8,
                    bytes: &[u8]
                ) -> Result<(), Self::Error>
                {
                    // Also wait until signalled we're master and everything is waiting for us
                    while {
                        let sr2 = self.i2c.sr2.read();
                        sr2.busy().bit_is_set()
                    } {}

                    // Send a START condition
                    self.i2c.cr1.modify(|_, w| unsafe {
                        w.pos()
                            .clear_bit()
                            .start()
                            .set_bit()
                    });

                    // Wait until START condition was generated
                    busy_wait!(self.i2c, sb);

                    // Set up current address, we're trying to talk to
                    self.i2c.dr.write(|w| unsafe { w.bits(u32::from(address) << 1) });

                    // Wait until address was sent
                    busy_wait!(self.i2c, addr);

                    // Clear addr flag by reading SR2 after SR1
                    self.i2c.sr2.read();

                    for byte in bytes {
                        // Wait until we're ready for sending
                        busy_wait!(self.i2c, tx_e);

                        // put byte on the wire
                        self.i2c.dr.write(|w| unsafe { w.bits(u32::from(*byte)) });

                        // While until byte is transferred
                        busy_wait!(self.i2c, btf);
                    }

                    // Send STOP condition
                    self.i2c.cr1.modify(|_, w| w.stop().set_bit());

                    // Fallthrough is success
                    Ok(())
                }
            }

            /*impl<PINS> WriteIter for I2c<$I2CX, PINS> {
                type Error = Error;

                //fn try_write_iter<B>(
                fn try_write<B>(
                    &mut self,
                    address: u8,
                    bytes: B
                ) -> Result<(), Self::Error> where
                    B: IntoIterator<Item = u8>,
                {
                    // TODO support transfers of more than 255 bytes
                    //assert!(bytes.len() < 256 && bytes.len() > 0);

                    // Send a START condition
                    self.i2c.cr1.modify(|_, w| w.start().set_bit());

                    // Wait until START condition was generated
                    busy_wait!(self.i2c, sb);

                    // Also wait until signalled we're master and everything is waiting for us
                    while {
                        let sr2 = self.i2c.sr2.read();
                        sr2.msl().bit_is_clear() && sr2.busy().bit_is_clear()
                    } {}

                    // Set up current address, we're trying to talk to
                    self.i2c.dr.write(|w| unsafe { w.bits(u32::from(address) << 1) });

                    // Wait until address was sent
                    busy_wait!(self.i2c, address);

                    // Clear condition by reading SR2
                    self.i2c.sr2.read();

                    for byte in bytes {
                        // Wait until we're ready for sending
                        busy_wait!(self.i2c, tx_e);

                        // put byte on the wire
                        self.i2c.dr.write(|w| { w.bits(u32::from(byte)) });

                        // While until byte is transferred
                        busy_wait!(self.i2c, btf);
                    }

                    // Send STOP condition
                    //self.i2c.cr1.modify(|_, w| w.stop().set_bit());

                    // Fallthrough is success
                    Ok(())
                }
            }*/

            impl<PINS> WriteRead for I2c<$I2CX, PINS> {
                type Error = Error;
                fn try_write_read(
                    &mut self,
                    address: u8,
                    bytes: &[u8],
                    buffer: &mut [u8]
                ) -> Result<(), Self::Error>
                {
                    self.try_write(address, bytes)?;
                    self.try_read(address, buffer)?;
                    Ok(())
                }
            }

            /*impl<PINS> WriteIterRead for I2c<$I2CX, PINS> {
                type Error = Error;
                fn try_write_iter_read<B>(
                    &mut self,
                    address: u8,
                    bytes: B,
                    buffer: &mut [u8],
                ) -> Result<(), Self::Error> where
                    B: IntoIterator<Item = u8>,
                {
                    self.try_write(address, bytes)?;
                    self.try_read(address, buffer)?;
                    Ok(())
                }
            }*/
        )+
    }
}

hal! {
    I2C1: (i2c1, i2c1en, i2c1rst),
    I2C2: (i2c2, i2c2en, i2c2rst),
}
