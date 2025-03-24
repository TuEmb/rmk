use embassy_stm32::gpio::{Input, Pull};
use embedded_hal::digital::{ErrorType, InputPin};
use core::future::Future;

use crate::hc4067::HcPin;

// Define an enum that can hold both HcPin and Input
pub enum AnyInputPin<'a> {
    Hc(HcPin<'a>),
    Gpio(Input<'a>),
}

impl<'a> ErrorType for AnyInputPin<'a> {
    type Error = core::convert::Infallible;
}

impl<'a> InputPin for AnyInputPin<'a> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        match self {
            AnyInputPin::Hc(pin) => pin.is_high(),
            AnyInputPin::Gpio(pin) => pin.is_high(),
        }
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        match self {
            AnyInputPin::Hc(pin) => pin.is_low(),
            AnyInputPin::Gpio(pin) => pin.is_low(),
        }
    }
}