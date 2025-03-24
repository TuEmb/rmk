#![no_std]
#![no_main]

use core::cell::RefCell;
use embassy_stm32::gpio::{Input, Level, Output, Pull};
use embassy_stm32::peripherals::{PA15, PB12, PB13, PB14, PB15};
use embassy_stm32::Peripheral;
use embedded_hal::digital::{ErrorType, InputPin};

/// Represents a single virtual input pin from the HC4067 multiplexer.
pub struct HcPin<'a> {
    mux: &'a Hc4067,
    channel: u8,
}

impl<'a> HcPin<'a> {
    pub fn new(mux: &'a Hc4067, channel: u8) -> Self {
        Self { mux, channel }
    }
}

impl<'a> ErrorType for HcPin<'a> {
    type Error = core::convert::Infallible;
}

impl<'a> InputPin for HcPin<'a> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.mux.read_pin(self.channel))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.mux.read_pin(self.channel))
    }
}

/// Represents the HC4067 multiplexer with input pins.
pub struct Hc4067 {
    s0: RefCell<Output<'static>>,
    s1: RefCell<Output<'static>>,
    s2: RefCell<Output<'static>>,
    s3: RefCell<Output<'static>>,
    com: Input<'static>,
}

impl Hc4067 {
    pub fn new(
        s0: impl Peripheral<P = PB12> + 'static,
        s1: impl Peripheral<P = PB13> + 'static,
        s2: impl Peripheral<P = PB15> + 'static,
        s3: impl Peripheral<P = PB14> + 'static,
        com: impl Peripheral<P = PA15> + 'static,
    ) -> Self {
        Self {
            s0: RefCell::new(Output::new(
                s0,
                Level::Low,
                embassy_stm32::gpio::Speed::High,
            )),
            s1: RefCell::new(Output::new(
                s1,
                Level::Low,
                embassy_stm32::gpio::Speed::High,
            )),
            s2: RefCell::new(Output::new(
                s2,
                Level::Low,
                embassy_stm32::gpio::Speed::High,
            )),
            s3: RefCell::new(Output::new(
                s3,
                Level::Low,
                embassy_stm32::gpio::Speed::High,
            )),
            com: Input::new(com, Pull::Up),
        }
    }

    /// Selects the active channel (0-15).
    fn select_channel(&self, channel: u8) {
        self.s0.borrow_mut().set_level(if channel & 0b0001 != 0 {
            Level::High
        } else {
            Level::Low
        });
        self.s1.borrow_mut().set_level(if channel & 0b0010 != 0 {
            Level::High
        } else {
            Level::Low
        });
        self.s2.borrow_mut().set_level(if channel & 0b0100 != 0 {
            Level::High
        } else {
            Level::Low
        });
        self.s3.borrow_mut().set_level(if channel & 0b1000 != 0 {
            Level::High
        } else {
            Level::Low
        });
    }

    /// Reads the value from a specific pin (0-15).
    pub fn read_pin(&self, pin: u8) -> bool {
        self.select_channel(pin);
        self.com.is_high()
    }

    /// Returns an instance of a virtual input pin.
    pub fn get_pin<'a>(&'a self, channel: u8) -> HcPin<'a> {
        HcPin::new(self, channel)
    }
}
