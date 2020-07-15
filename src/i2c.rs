//! I2C

use crate::stm32::{I2C1, I2C2};
use cast::u8;

use crate::gpio::gpiob::{PB10, PB11, PB6, PB7, PB8, PB9};
use crate::gpio::AF4;
use crate::rcc::{Clocks, APB1};
use crate::time::Hertz;
use hal::blocking::i2c::{Write, WriteRead};

/// I2C error
#[derive(Debug)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
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

unsafe impl SclPin<I2C1> for PB6<AF4> {}
unsafe impl SclPin<I2C1> for PB8<AF4> {}

unsafe impl SclPin<I2C2> for PB10<AF4> {}

unsafe impl SdaPin<I2C1> for PB7<AF4> {}
unsafe impl SdaPin<I2C1> for PB9<AF4> {}

unsafe impl SdaPin<I2C2> for PB11<AF4> {}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident) => {
        loop {
            let isr = $i2c.isr.read();

            if isr.berr().bit_is_set() {
                return Err(Error::Bus);
            } else if isr.arlo().bit_is_set() {
                return Err(Error::Arbitration);
            } else if isr.$flag().bit_is_set() {
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
                    freq: F,
                    clocks: Clocks,
                    apb1: &mut APB1,
                ) -> Self where
                    F: Into<Hertz>,
                    SCL: SclPin<$I2CX>,
                    SDA: SdaPin<$I2CX>,
                {
                    apb1.enr().modify(|_, w| w.$i2cXen().enabled());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().set_bit());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().clear_bit());

                    let freq = freq.into().0;

                    assert!(freq <= 1_000_000);

                    // TODO review compliance with the timing requirements of I2C
                    // t_I2CCLK = 1 / PCLK1
                    // t_PRESC  = (PRESC + 1) * t_I2CCLK
                    // t_SCLL   = (SCLL + 1) * t_PRESC
                    // t_SCLH   = (SCLH + 1) * t_PRESC
                    //
                    // t_SYNC1 + t_SYNC2 > 4 * t_I2CCLK
                    // t_SCL ~= t_SYNC1 + t_SYNC2 + t_SCLL + t_SCLH
                    let i2cclk = clocks.pclk1().0;
                    let ratio = i2cclk / freq - 4;
                    let (presc, scll, sclh, sdadel, scldel) = if freq >= 100_000 {
                        // fast-mode or fast-mode plus
                        // here we pick SCLL + 1 = 2 * (SCLH + 1)
                        let presc = ratio / 387;

                        let sclh = ((ratio / (presc + 1)) - 3) / 3;
                        let scll = 2 * (sclh + 1) - 1;

                        let (sdadel, scldel) = if freq > 400_000 {
                            // fast-mode plus
                            let sdadel = 0;
                            let scldel = i2cclk / 4_000_000 / (presc + 1) - 1;

                            (sdadel, scldel)
                        } else {
                            // fast-mode
                            let sdadel = i2cclk / 8_000_000 / (presc + 1);
                            let scldel = i2cclk / 2_000_000 / (presc + 1) - 1;

                            (sdadel, scldel)
                        };

                        (presc, scll, sclh, sdadel, scldel)
                    } else {
                        // standard-mode
                        // here we pick SCLL = SCLH
                        let presc = ratio / 514;

                        let sclh = ((ratio / (presc + 1)) - 2) / 2;
                        let scll = sclh;

                        let sdadel = i2cclk / 2_000_000 / (presc + 1);
                        let scldel = i2cclk / 800_000 / (presc + 1) - 1;

                        (presc, scll, sclh, sdadel, scldel)
                    };

                    let presc = u8(presc).unwrap();
                    assert!(presc < 16);
                    let scldel = u8(scldel).unwrap();
                    assert!(scldel < 16);
                    let sdadel = u8(sdadel).unwrap();
                    assert!(sdadel < 16);
                    let sclh = u8(sclh).unwrap();
                    let scll = u8(scll).unwrap();

                    // Configure for "fast mode" (400 KHz)
                    i2c.timingr.write(|w| unsafe {
                        w.presc()
                            .bits(presc)
                            .scll()
                            .bits(scll)
                            .sclh()
                            .bits(sclh)
                            .sdadel()
                            .bits(sdadel)
                            .scldel()
                            .bits(scldel)
                    });

                    // Enable the peripheral
                    i2c.cr1.write(|w| w.pe().set_bit());

                    I2c { i2c, pins }
                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, (SCL, SDA)) {
                    (self.i2c, self.pins)
                }
            }

            impl<PINS> Write for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);

                    // START and prepare to send `bytes`
                    self.i2c.cr2.write(|w| {
                        w.sadd1()
                            .bits(addr)
                            .rd_wrn()
                            .clear_bit()
                            .nbytes()
                            .bits(bytes.len() as u8)
                            .start()
                            .set_bit()
                            .autoend()
                            .set_bit()
                    });

                    for byte in bytes {
                        // Wait until we are allowed to send data (START has been ACKed or last byte
                        // when through)
                        busy_wait!(self.i2c, txis);

                        // put byte on the wire
                        self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                    }

                    // Wait until the last transmission is finished ???
                    // busy_wait!(self.i2c, busy);

                    // automatic STOP

                    Ok(())
                }
            }

            impl<PINS> WriteRead for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write_read(
                    &mut self,
                    addr: u8,
                    bytes: &[u8],
                    buffer: &mut [u8],
                ) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);
                    assert!(buffer.len() < 256 && buffer.len() > 0);

                    // TODO do we have to explicitly wait here if the bus is busy (e.g. another
                    // master is communicating)?

                    // START and prepare to send `bytes`
                    self.i2c.cr2.write(|w| {
                        w.sadd1()
                            .bits(addr)
                            .rd_wrn()
                            .clear_bit()
                            .nbytes()
                            .bits(bytes.len() as u8)
                            .start()
                            .set_bit()
                            .autoend()
                            .clear_bit()
                    });

                    for byte in bytes {
                        // Wait until we are allowed to send data (START has been ACKed or last byte
                        // when through)
                        busy_wait!(self.i2c, txis);

                        // put byte on the wire
                        self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                    }

                    // Wait until the last transmission is finished
                    busy_wait!(self.i2c, tc);

                    // reSTART and prepare to receive bytes into `buffer`
                    self.i2c.cr2.write(|w| {
                        w.sadd1()
                            .bits(addr)
                            .rd_wrn()
                            .set_bit()
                            .nbytes()
                            .bits(buffer.len() as u8)
                            .start()
                            .set_bit()
                            .autoend()
                            .set_bit()
                    });

                    for byte in buffer {
                        // Wait until we have received something
                        busy_wait!(self.i2c, rxne);

                        *byte = self.i2c.rxdr.read().rxdata().bits();
                    }

                    // automatic STOP

                    Ok(())
                }
            }
        )+
    }
}

macro_rules! i2c {
    ($I2CX:ident, $i2cx:ident, $i2cxen:ident, $i2crst:ident) => {
        impl<PINS> I2c<$I2CX, PINS> {
            pub fn $i2cx(i2c: $I2CX, pins: PINS, speed: Hertz, rcc: &mut Rcc) -> Self
            where
                PINS: Pins<$I2CX>,
            {
                pins.setup();
                let speed: Hertz = speed.into();

                // Enable clock for I2C
                rcc.rb.apb1enr.modify(|_, w| w.$i2cxen().set_bit());

                // Reset I2C
                rcc.rb.apb1rstr.modify(|_, w| w.$i2crst().set_bit());
                rcc.rb.apb1rstr.modify(|_, w| w.$i2crst().clear_bit());

                // Make sure the I2C unit is disabled so we can configure it
                i2c.cr1.modify(|_, w| w.pe().clear_bit());

                // Calculate settings for I2C speed modes
                let clock = rcc.clocks.apb1_clk().0;
                let freq = clock / 1_000_000;
                assert!(freq >= 2 && freq <= 50);

                // Configure bus frequency into I2C peripheral
                i2c.cr2.write(|w| unsafe { w.freq().bits(freq as u8) });

                let trise = if speed <= 100_u32.khz().into() {
                    freq + 1
                } else {
                    (freq * 300) / 1000 + 1
                };

                // Configure correct rise times
                i2c.trise.write(|w| w.trise().bits(trise as u8));

                // I2C clock control calculation
                if speed <= 100_u32.khz().into() {
                    let ccr = {
                        let ccr = clock / (speed.0 * 2);
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
                        let ccr = clock / (speed.0 * 3);
                        let ccr = if ccr < 1 { 1 } else { ccr };

                        // Set clock to fast mode with appropriate parameters for selected speed (2:1 duty cycle)
                        i2c.ccr.write(|w| unsafe {
                            w.f_s().set_bit().duty().clear_bit().ccr().bits(ccr as u16)
                        });
                    } else {
                        let ccr = clock / (speed.0 * 25);
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

            pub fn release(self) -> ($I2CX, PINS) {
                (self.i2c, self.pins)
            }

            fn send_byte(&self, byte: u8) -> Result<(), Error> {
                // Wait until we're ready for sending
                while self.i2c.sr1.read().tx_e().bit_is_clear() {}

                // Push out a byte of data
                self.i2c.dr.write(|w| unsafe { w.bits(u32::from(byte)) });

                // While until byte is transferred
                while {
                    let sr1 = self.i2c.sr1.read();

                    // If we received a NACK, then this is an error
                    if sr1.af().bit_is_set() {
                        return Err(Error::NACK);
                    }

                    sr1.btf().bit_is_clear()
                } {}

                Ok(())
            }

            fn recv_byte(&self) -> Result<u8, Error> {
                while self.i2c.sr1.read().rx_ne().bit_is_clear() {}
                let value = self.i2c.dr.read().bits() as u8;
                Ok(value)
            }
        }

        impl<PINS> WriteRead for I2c<$I2CX, PINS> {
            type Error = Error;

            fn write_read(
                &mut self,
                addr: u8,
                bytes: &[u8],
                buffer: &mut [u8],
            ) -> Result<(), Self::Error> {
                self.write(addr, bytes)?;
                self.read(addr, buffer)?;

                Ok(())
            }
        }

        impl<PINS> Write for I2c<$I2CX, PINS> {
            type Error = Error;

            fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
                // Send a START condition
                self.i2c.cr1.modify(|_, w| w.start().set_bit());

                // Wait until START condition was generated
                while {
                    let sr1 = self.i2c.sr1.read();
                    sr1.sb().bit_is_clear()
                } {}

                // Also wait until signalled we're master and everything is waiting for us
                while {
                    let sr2 = self.i2c.sr2.read();
                    sr2.msl().bit_is_clear() && sr2.busy().bit_is_clear()
                } {}

                // Set up current address, we're trying to talk to
                self.i2c
                    .dr
                    .write(|w| unsafe { w.bits(u32::from(addr) << 1) });

                // Wait until address was sent
                while {
                    let sr1 = self.i2c.sr1.read();
                    sr1.addr().bit_is_clear()
                } {}

                // Clear condition by reading SR2
                self.i2c.sr2.read();

                // Send bytes
                for c in bytes {
                    self.send_byte(*c)?;
                }

                // Fallthrough is success
                Ok(())
            }
        }

        impl<PINS> Read for I2c<$I2CX, PINS> {
            type Error = Error;

            fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
                // Send a START condition and set ACK bit
                self.i2c
                    .cr1
                    .modify(|_, w| w.start().set_bit().ack().set_bit());

                // Wait until START condition was generated
                while {
                    let sr1 = self.i2c.sr1.read();
                    sr1.sb().bit_is_clear()
                } {}

                // Also wait until signalled we're master and everything is waiting for us
                while {
                    let sr2 = self.i2c.sr2.read();
                    sr2.msl().bit_is_clear() && sr2.busy().bit_is_clear()
                } {}

                // Set up current address, we're trying to talk to
                self.i2c
                    .dr
                    .write(|w| unsafe { w.bits((u32::from(addr) << 1) + 1) });

                // Wait until address was sent
                while {
                    let sr1 = self.i2c.sr1.read();
                    sr1.addr().bit_is_clear()
                } {}

                // Clear condition by reading SR2
                self.i2c.sr2.read();

                // Receive bytes into buffer
                for c in buffer {
                    *c = self.recv_byte()?;
                }

                // Send STOP condition
                self.i2c.cr1.modify(|_, w| w.stop().set_bit());

                // Fallthrough is success
                Ok(())
            }
        }

        impl I2cExt<$I2CX> for $I2CX {
            fn i2c<PINS, T>(self, pins: PINS, speed: T, rcc: &mut Rcc) -> I2c<$I2CX, PINS>
            where
                PINS: Pins<$I2CX>,
                T: Into<Hertz>,
            {
                I2c::$i2cx(self, pins, speed.into(), rcc)
            }
        }
    };
}

pub trait I2cExt<I2C> {
    fn i2c<PINS, T>(self, pins: PINS, speed: T, rcc: &mut Rcc) -> I2c<I2C, PINS>
    where
        PINS: Pins<I2C>,
        T: Into<Hertz>;
}

i2c!(I2C1, i2c1, i2c1en, i2c1rst);
i2c!(I2C2, i2c2, i2c2en, i2c2rst);
