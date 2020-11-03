//! Reset and Clock Control

//use crate::flash::ACR;
use crate::stm32::{rcc, RCC, pwr, PWR};
use crate::time::Hertz;
use cast::u32;


#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MsiFreq {
    #[doc = "range 0 around 65536 Hz"]
    RANGE_65536HZ = 0,
    #[doc = "range 1 around 131072 Hz"]
    RANGE_131072HZ = 1,
    #[doc = "range 2 around 262144 Hz"]
    RANGE_262144HZ = 2,
    #[doc = "range 3 around 524288 Hz"]
    RANGE_524288HZ = 3,
    #[doc = "range 4 around 1048000 Hz"]
    RANGE_1048000HZ = 4,
    #[doc = "range 5 around 2097000 Hz"]
    RANGE_2097000HZ = 5,
    #[doc = "range 6 around 4194000 Hz"]
    RANGE_4194000HZ = 6,
}

/*impl MsiFreq {
    fn to_hertz(self) -> Hertz {
        Hertz(match self {
            Self::RANGE_65536HZ => 65_536,
            Self::RANGE_131072HZ => 131_072,
            Self::RANGE_262144HZ => 262_144,
            Self::RANGE_524288HZ => 524_288,
            Self::RANGE_1048000HZ => 1_048_000,
            Self::RANGE_2097000HZ => 2_097_000,
            Self::RANGE_4194000HZ => 4_194_000,
        })
    }
}*/

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb: AHB { _0: () },
            apb1: APB1 { _0: () },
            apb2: APB2 { _0: () },
            csr: CSR { _0: () },
            cfgr: CFGR {
                hse: None,
                lse: None,
                msi: None,
                lsi: false,
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: None,
                pll_source: None,
                pll_config: None,
            },
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// AMBA High-performance Bus (AHB) registers
    pub ahb: AHB,
    /// Advanced Peripheral Bus 1 (APB1) registers
    pub apb1: APB1,
    /// Advanced Peripheral Bus 2 (APB2) registers
    pub apb2: APB2,
    /// Clock configuration register
    pub cfgr: CFGR,
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
    pub(crate) fn csr(&mut self) -> &rcc::CSR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).csr }
    }
}

/// AMBA High-performance Bus 1 (AHB) registers
pub struct AHB {
    _0: (),
}

impl AHB {
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn enr(&mut self) -> &rcc::AHBENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahbenr }
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn rstr(&mut self) -> &rcc::AHBRSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahbrstr }
    }
}

/// Advanced Peripheral Bus 1 (APB1) registers
pub struct APB1 {
    _0: (),
}

impl APB1 {
    pub(crate) fn enr(&mut self) -> &rcc::APB1ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB1RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1rstr }
    }
}

/// Advanced Peripheral Bus 2 (APB2) registers
pub struct APB2 {
    _0: (),
}

impl APB2 {
    pub(crate) fn enr(&mut self) -> &rcc::APB2ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB2RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2rstr }
    }
}

#[derive(Debug, PartialEq)]
/// HSE Configuration
struct HseConfig {
    /// Clock speed of HSE
    speed: u32,
    /// If the clock driving circuitry is bypassed i.e. using an oscillator, not a crystal or
    /// resonator
    bypass: CrystalBypass,
    /// Clock Security System enable/disable
    css: ClockSecuritySystem,
}

#[derive(Debug, PartialEq)]
/// LSE Configuration
struct LseConfig {
    /// If the clock driving circuitry is bypassed i.e. using an oscillator, not a crystal or
    /// resonator
    bypass: CrystalBypass,
}

/// Crystal bypass selector
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CrystalBypass {
    /// If the clock driving circuitry is bypassed i.e. using an oscillator
    Enable,
    /// If the clock driving circuitry is not bypassed i.e. using a crystal or resonator
    Disable,
}

/// Clock Security System (CSS) selector
///
/// When this is enabled on HSE it will fire of the NMI interrupt on failure and for the LSE the
/// MCU will be woken if in Standby and then the LSECSS interrupt will fire. See datasheet on how
/// to recover for CSS failures.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ClockSecuritySystem {
    /// Enable the clock security system to detect clock failures
    Enable,
    /// Leave the clock security system disabled
    Disable,
}

const HSI: u32 = 16_000_000; // Hz

/// Clock configuration
pub struct CFGR {
    hse: Option<HseConfig>,
    lse: Option<LseConfig>,
    msi: Option<MsiFreq>,
    lsi: bool,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
    pll_source: Option<PllSource>,
    pll_config: Option<PllConfig>,
}

impl CFGR {
    /// Add an HSE to the system
    pub fn hse<F>(mut self, freq: F, bypass: CrystalBypass, css: ClockSecuritySystem) -> Self
    where
        F: Into<Hertz>,
    {
        self.hse = Some(HseConfig {
            speed: freq.into().0,
            bypass: bypass,
            css: css,
        });

        self
    }

    /// Add an 32.768 kHz LSE to the system
    pub fn lse(mut self, bypass: CrystalBypass) -> Self {
        self.lse = Some(LseConfig { bypass: bypass });

        self
    }

    /// Sets a frequency for the AHB bus
    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hclk = Some(freq.into().0);
        self
    }

    /// Enables the MSI with the specified speed
    pub fn msi(mut self, range: MsiFreq) -> Self {
        self.msi = Some(range);
        self
    }

    /// Sets LSI clock on (the default) or off
    pub fn lsi(mut self, on: bool) -> Self {
        self.lsi = on;
        self
    }

    /// Sets a frequency for the APB1 bus
    pub fn pclk1<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk1 = Some(freq.into().0);
        self
    }

    /// Sets a frequency for the APB2 bus
    pub fn pclk2<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk2 = Some(freq.into().0);
        self
    }

    /// Sets the system (core) frequency
    pub fn sysclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.sysclk = Some(freq.into().0);
        self
    }

    /// Sets the system (core) frequency with some pll configuration
    pub fn sysclk_with_pll<F>(mut self, freq: F, cfg: PllConfig) -> Self
    where
        F: Into<Hertz>,
    {
        self.pll_config = Some(cfg);
        self.sysclk = Some(freq.into().0);
        self
    }

    /// Sets the PLL source
    pub fn pll_source(mut self, source: PllSource) -> Self {
        self.pll_source = Some(source);
        self
    }

    /// Freezes the clock configuration, making it effective
    pub fn freeze(&self /*, acr: &mut ACR*/) -> Clocks {
        let rcc = unsafe { &*RCC::ptr() };

        //
        // 1. Setup clocks
        //

        // Turn on the internal 32 kHz LSI oscillator
        let lsi_used = match (self.lsi, &self.lse) {
            (true, _) | (_, &Some(LseConfig { bypass: _ })) => {
                rcc.csr.modify(|_, w| w.lsion().set_bit());

                // Wait until LSI is running
                while rcc.csr.read().lsirdy().bit_is_clear() {}

                true
            }
            _ => false,
        };

        if let Some(lse_cfg) = &self.lse {
            // 1. Unlock the backup domain
            //pwr.cr1.reg().modify(|_, w| w.dbp().set_bit());

            // 2. Setup the LSE
            rcc.csr.modify(|_, w| {
                w.lseon().set_bit(); // Enable LSE

                if lse_cfg.bypass == CrystalBypass::Enable {
                    w.lsebyp().set_bit();
                }

                w
            });

            // Wait until LSE is running
            while rcc.csr.read().lserdy().bit_is_clear() {}
        }

        // If HSE is available, set it up
        if let Some(hse_cfg) = &self.hse {
            rcc.cr.write(|w| {
                w.hseon().set_bit();

                if hse_cfg.bypass == CrystalBypass::Enable {
                    w.hsebyp().set_bit();
                }

                w
            });

            while rcc.cr.read().hserdy().bit_is_clear() {}

            // Setup CSS
            if hse_cfg.css == ClockSecuritySystem::Enable {
                // Enable CSS
                rcc.cr.modify(|_, w| w.csson().set_bit());
            }
        }

        if let Some(msi) = self.msi {
            rcc.icscr
                .modify(|_, w| unsafe { w.msirange().bits(msi as u8) });
            rcc.cr.modify(|_, w| w.msion().set_bit());

            // Wait until MSI is running
            while rcc.cr.read().msirdy().bit_is_clear() {}
        }

        //
        // 2. Setup PLL
        //

        // Select PLL source
        let (clock_speed, pll_source) = if let Some(source) = self.pll_source {
            match source {
                PllSource::HSE => {
                    if let Some(hse) = &self.hse {
                        (hse.speed, source)
                    } else {
                        panic!("HSE selected as PLL source, but not enabled");
                    }
                }
                PllSource::HSI16 => (HSI, source),
            }
        } else {
            // No specific PLL source selected, do educated guess

            // 1. HSE
            if let Some(hse) = &self.hse {
                (hse.speed, PllSource::HSE)
            }
            // 2. HSI as fallback
            else {
                (HSI, PllSource::HSI16)
            }
        };

        // Check if HSI should be started
        if pll_source == PllSource::HSI16 || self.hse.is_none() {
            rcc.cr.write(|w| w.hsion().set_bit());
            while rcc.cr.read().hsirdy().bit_is_clear() {}
        }

        let pllconf = if self.pll_config.is_none() {
            //if let Some(sysclk) = self.sysclk {
            if let Some(_) = self.sysclk {
                // Calculate PLL multiplier and create a best effort pll config, just multiply n
                //let plln = (2 * sysclk) / clock_speed;

                // TODO pllmul
                Some(PllConfig::new(PllMultiplier::Mul4, PllDivider::Div4))
            } else {
                None
            }
        } else {
            self.pll_config
        };

        let sysclk = self.sysclk.unwrap_or(HSI);

        assert!(sysclk <= 32_000_000);

        let (hpre_bits, hpre_div) = self
            .hclk
            .map(|hclk| match sysclk / hclk {
                // From p 194 in RM0394
                0 => unreachable!(),
                1 => (0b0000, 1),
                2 => (0b1000, 2),
                3..=5 => (0b1001, 4),
                6..=11 => (0b1010, 8),
                12..=39 => (0b1011, 16),
                40..=95 => (0b1100, 64),
                96..=191 => (0b1101, 128),
                192..=383 => (0b1110, 256),
                _ => (0b1111, 512),
            })
            .unwrap_or((0b0000, 1));

        let hclk = sysclk / hpre_div;

        assert!(hclk <= sysclk);

        let (ppre1_bits, ppre1) = self
            .pclk1
            .map(|pclk1| match hclk / pclk1 {
                // From p 194 in RM0394
                0 => unreachable!(),
                1 => (0b000, 1),
                2 => (0b100, 2),
                3..=5 => (0b101, 4),
                6..=11 => (0b110, 8),
                _ => (0b111, 16),
            })
            .unwrap_or((0b000, 1));

        let pclk1 = hclk / u32(ppre1);

        assert!(pclk1 <= sysclk);

        let (ppre2_bits, ppre2) = self
            .pclk2
            .map(|pclk2| match hclk / pclk2 {
                // From p 194 in RM0394
                0 => unreachable!(),
                1 => (0b000, 1),
                2 => (0b100, 2),
                3..=5 => (0b101, 4),
                6..=11 => (0b110, 8),
                _ => (0b111, 16),
            })
            .unwrap_or((0b000, 1));

        let pclk2 = hclk / u32(ppre2);

        assert!(pclk2 <= sysclk);

        // adjust flash wait states
        /*unsafe { acr.acr().write(|w| w.acc64().set_bit()) }

        // adjust flash wait states
        unsafe {
            acr.acr().write(|w| {
                if sysclk <= 24_000_000 {
                    w.latency().clear_bit()
                } else {
                    w.latency().set_bit()
                }
            })
        }

        // adjust flash wait states
        unsafe { acr.acr().write(|w| w.acc64().clear_bit()) }*/

        let sysclk_src_bits;
        // if let Some(pllconf) = pllconf {
        //     // Sanity-checks per RM0394, 6.4.4 PLL configuration register (RCC_PLLCFGR)
        //     //let r = pllconf.r.to_division_factor();
        //     //let clock_speed = clock_speed / (pllconf.m as u32 + 1);
        //     let vco = clock_speed * pllconf.pllmul.to_multiplication_factor();
        //     let output_clock = vco / pllconf.plldiv.to_division_factor();

        //     assert!(clock_speed >= 2_000_000); // VCO input clock min
        //     assert!(clock_speed <= 24_000_000); // VCO input clock max
        //     assert!(vco >= 6_000_000); // VCO output min
        //     assert!(vco <= 96_000_000); // VCO output max
        //     assert!(output_clock <= 32_000_000); // Max output clock

        //     // use PLL as source
        //     sysclk_src_bits = 0b11;
        //     rcc.cr.modify(|_, w| w.pllon().clear_bit());
        //     while rcc.cr.read().pllrdy().bit_is_set() {}

        //     let pllsrc_bit = pll_source.to_pllsrc();

        //     rcc.cfgr.modify(|_, w| unsafe {
        //         w.pllsrc()
        //             .bit(pllsrc_bit)
        //             .plldiv()
        //             .bits(pllconf.plldiv.to_bits())
        //             .pllmul()
        //             .bits(pllconf.pllmul.to_bits())
        //     });

        //     rcc.cr.modify(|_, w| w.pllon().set_bit());

        //     while rcc.cr.read().pllrdy().bit_is_clear() {}

        //     //rcc.pllcfgr.modify(|_, w| w.pllren().set_bit());

        //     // SW: PLL selected as system clock
        //     rcc.cfgr.modify(|_, w| unsafe {
        //         w.ppre2()
        //             .bits(ppre2_bits)
        //             .ppre1()
        //             .bits(ppre1_bits)
        //             .hpre()
        //             .bits(hpre_bits)
        //             .sw()
        //             .bits(sysclk_src_bits)
        //     });
        // } else {
        //     // use HSI as source
        //     sysclk_src_bits = 0b01;

        //     rcc.cr.write(|w| w.hsion().set_bit());
        //     while rcc.cr.read().hsirdy().bit_is_clear() {}

        //     // SW: HSI selected as system clock
        //     rcc.cfgr.write(|w| unsafe {
        //         w.ppre2()
        //             .bits(ppre2_bits)
        //             .ppre1()
        //             .bits(ppre1_bits)
        //             .hpre()
        //             .bits(hpre_bits)
        //             .sw()
        //             .bits(sysclk_src_bits)
        //     });
        // }

        rcc.csr.modify(|_, w| w.lsion().set_bit());

        // Wait until LSI is running
        while rcc.csr.read().lsirdy().bit_is_clear() {}

        rcc.apb1enr.modify(|_r, w| w.pwren().set_bit());
        // Enable access to the backup registers
        let pwr = unsafe { &*PWR::ptr() };
        pwr.cr.modify(|_r, w| w.dbp().set_bit());
        rcc.csr.modify(|_, w| unsafe { w.rtcsel().bits(0b10) });
        rcc.apb1enr.modify(|_r, w| w.lcden().set_bit());

        sysclk_src_bits = 0b00;

        // SW: HSI selected as system clock
        rcc.cfgr.write(|w| unsafe {
            w.ppre2()
                .bits(ppre2_bits)
                .ppre1()
                .bits(ppre1_bits)
                .hpre()
                .bits(hpre_bits)
                .sw()
                .bits(sysclk_src_bits)
        });

        while rcc.cfgr.read().sws().bits() != sysclk_src_bits {}

        //
        // 3. Shutdown unused clocks that have auto-started
        //

        // MSI always starts on reset
        /*if self.msi.is_none() {
            rcc.cr.modify(|_, w| w.msion().clear_bit())
        }*/

        //
        // 4. Clock setup done!
        //

        Clocks {
            hclk: Hertz(hclk),
            lsi: lsi_used,
            lse: self.lse.is_some(),
            msi: self.msi,
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            ppre1: ppre1,
            ppre2: ppre2,
            sysclk: Hertz(sysclk),
            pll_source: pllconf.map(|_| pll_source),
        }
    }
}

#[derive(Clone, Copy, Debug)]
/// PLL output multiplier options
pub enum PllMultiplier {
    /// Multiplier PLL output by 3
    Mul3 = 0b0000,
    /// Multiplier PLL output by 4
    Mul4 = 0b0001,
    /// Multiplier PLL output by 6
    Mul6 = 0b0010,
    /// Multiplier PLL output by 8
    Mul8 = 0b0011,
    /// Multiplier PLL output by 12
    Mul12 = 0b0100,
    /// Multiplier PLL output by 16
    Mul16 = 0b0101,
    /// Multiplier PLL output by 24
    Mul24 = 0b0110,
    /// Multiplier PLL output by 32
    Mul32 = 0b0111,
    /// Multiplier PLL output by 48
    Mul48 = 0b1000,
}

impl PllMultiplier {
    #[inline(always)]
    fn to_bits(self) -> u8 {
        self as u8
    }

    #[inline(always)]
    fn to_multiplication_factor(self) -> u32 {
        match self {
            Self::Mul3 => 3,
            Self::Mul4 => 4,
            Self::Mul6 => 6,
            Self::Mul8 => 8,
            Self::Mul12 => 12,
            Self::Mul16 => 16,
            Self::Mul24 => 24,
            Self::Mul32 => 32,
            Self::Mul48 => 48,
        }
    }
}

#[derive(Clone, Copy, Debug)]
/// PLL output divider options
pub enum PllDivider {
    /// Divider PLL output by 2
    Div2 = 0b00,
    /// Divider PLL output by 3
    Div3 = 0b01,
    /// Divider PLL output by 4
    Div4 = 0b10,
}

impl PllDivider {
    #[inline(always)]
    fn to_bits(self) -> u8 {
        self as u8
    }

    #[inline(always)]
    fn to_division_factor(self) -> u32 {
        match self {
            Self::Div2 => 2,
            Self::Div3 => 3,
            Self::Div4 => 4,
        }
    }
}

#[derive(Clone, Copy, Debug)]
/// PLL Configuration
pub struct PllConfig {
    // Main PLL multiplication factor
    pllmul: PllMultiplier,
    // Main PLL division factor
    plldiv: PllDivider,
}

impl PllConfig {
    /// Create a new PLL config from manual settings
    ///
    /// PLL output = (SourceClk * multiplier) / divider
    pub fn new(multiplier: PllMultiplier, divider: PllDivider) -> Self {
        PllConfig {
            pllmul: multiplier,
            plldiv: divider,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// PLL Source
pub enum PllSource {
    /// High-speed internal clock
    HSI16,
    /// High-speed external clock
    HSE,
}

impl PllSource {
    fn to_pllsrc(self) -> bool {
        match self {
            Self::HSI16 => false,
            Self::HSE => true,
        }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[derive(Clone, Copy, Debug)]
pub struct Clocks {
    hclk: Hertz,
    msi: Option<MsiFreq>,
    lsi: bool,
    lse: bool,
    pclk1: Hertz,
    pclk2: Hertz,
    ppre1: u8,
    ppre2: u8,
    sysclk: Hertz,
    pll_source: Option<PllSource>,
}

impl Clocks {
    /// Returns the frequency of the AHB
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    // Returns the status of the MSI
    pub fn msi(&self) -> Option<MsiFreq> {
        self.msi
    }

    /// Returns status of HSI48
    pub fn lsi(&self) -> bool {
        self.lsi
    }

    /// Returns the frequency of the APB1
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB2
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn ppre1(&self) -> u8 {
        self.ppre1
    }
    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn ppre2(&self) -> u8 {
        self.ppre2
    }

    /// Returns the system (core) frequency
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }
}
