use cortex_m::prelude::_embedded_hal_blocking_spi_Write;
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::{Spi, Config};
use embassy_stm32::peripherals::SPI1;
use embassy_stm32::time::mhz;
use embedded_hal::spi::SpiDevice;
use defmt::*;

pub struct Ws2812<'a> {
    spi: Spi<'a, Blocking>,
}

impl<'a> Ws2812<'a> {
    pub fn new(spi: Spi<'a, Blocking>) -> Self {
        Self { spi }
    }

    fn encode_ws2812_byte(byte: u8) -> [u8; 8] {
        let mut encoded = [0u8; 8];
        for i in 0..8 {
            if (byte & (1 << (7 - i))) != 0 {
                encoded[i] = 0b11111000; // WS2812 "1"
            } else {
                encoded[i] = 0b00110000; // WS2812 "0"
            }
        }
        encoded
    }

    pub async fn set_leds(&mut self, leds: &[[u8; 3]]) {
        let mut buffer = [0u8; 24 * 5]; // 24 WS2812 bits per LED (8x3)
        for (i, &rgb) in leds.iter().enumerate() {
            let offset = i * 24;
            buffer[offset..offset + 8].copy_from_slice(&Self::encode_ws2812_byte(rgb[1])); // Green
            buffer[offset + 8..offset + 16].copy_from_slice(&Self::encode_ws2812_byte(rgb[0])); // Red
            buffer[offset + 16..offset + 24].copy_from_slice(&Self::encode_ws2812_byte(rgb[2])); // Blue
        }

        // Send SPI data
        self.spi.write(&mut buffer).ok();
        
        // Delay 50µs (reset WS2812)
        embassy_time::Timer::after_micros(50).await;
    }
}


