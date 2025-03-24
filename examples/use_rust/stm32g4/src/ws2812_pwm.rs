use embassy_stm32::peripherals::{TIM1, TIM17};
use embassy_stm32::timer::simple_pwm::{Ch1, PwmPin, SimplePwm};
use embassy_stm32::time::mhz;
use embassy_stm32::PeripheralRef;

// WS2812 timing constants
const WS2812_T1H: u32 = 800;  // 800ns
const WS2812_T0H: u32 = 400;  // 400ns
const WS2812_PERIOD: u32 = 1250; // Total period (1.25µs per bit)

pub struct Ws2812Pwm<'a> {
    pwm: SimplePwm<'a, TIM17>,
}

impl<'a> Ws2812Pwm<'a>{
    pub fn new(timer: TIM17, pin: PwmPin<'a, TIM17, Ch1>) -> Self {
        let pwm = SimplePwm::new(timer, Some(pin), None, None, None, mhz(8), Default::default()); // 8 MHz base clock
        Self { pwm }
    }

    pub async  fn send(&mut self, data: &[u8]) {
        let mut pwm_buffer = [0u16; 24 * 5]; // 5 LEDs, 24 bits per LED

        for (i, byte) in data.iter().enumerate() {
            for bit in 0..8 {
                let high_time = if (byte & (1 << (7 - bit))) != 0 {
                    WS2812_T1H
                } else {
                    WS2812_T0H
                };

                pwm_buffer[i * 8 + bit] = high_time as u16;
            }
        }

        // Send data using DMA
        self.pwm.ch1().set_duty_cycle_fraction(8, 10); // First pulse
        for &duty in &pwm_buffer[1..] {
            self.pwm.ch1().set_duty_cycle_fraction(0, duty);
        }

        // Reset: Keep LOW for at least 50µs
        embassy_time::Timer::after_micros(50).await;
    }
}
