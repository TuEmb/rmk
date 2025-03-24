#![no_main]
#![no_std]

#[macro_use]
mod macros;
mod hc4067;
mod keymap;
mod vial;
mod common_input;
mod is31fl3733;
mod ws2812;
mod ws2812_pwm;

use common_input::AnyInputPin;
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts, flash::Flash, gpio::{AnyPin, Input, Output, OutputType, Pin}, i2c::I2c, mode::{Async, Blocking}, peripherals::{self, RNG, USB}, rng::Rng, time::Hertz, timer::simple_pwm::PwmPin, usb::{Driver, InterruptHandler}, Config
};
use hc4067::{Hc4067, HcPin};
use is31fl3733::IS31FL3733;
use keymap::{COL, ROW, SIZE};
use panic_halt as _;
use rmk::{
    channel::EVENT_CHANNEL,
    config::{ControllerConfig, KeyboardUsbConfig, RmkConfig, VialConfig},
    debounce::default_debouncer::DefaultDebouncer,
    direct_pin::DirectPinMatrix,
    futures::future::join3,
    initialize_keymap_and_storage,
    input_device::Runnable,
    keyboard::Keyboard,
    light::LightController,
    run_devices, run_rmk,
    storage::async_flash_wrapper,
};
use vial::{VIAL_KEYBOARD_DEF, VIAL_KEYBOARD_ID};
use ws2812::Ws2812;
use ws2812_pwm::Ws2812Pwm;
bind_interrupts!(struct Irqs {
    USB_LP => InterruptHandler<USB>;
    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<peripherals::I2C1>;
    RNG => embassy_stm32::rng::InterruptHandler<peripherals::RNG>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("RMK start!");
    // RCC config
    let config = Config::default();
    // Initialize peripherals
    info!("embassy init");
    let p = embassy_stm32::init(config);
    let mut led = Output::new(
        p.PB11,
        embassy_stm32::gpio::Level::High,
        embassy_stm32::gpio::Speed::Low,
    );
    led.set_high();

    let mut spi_config = embassy_stm32::spi::Config::default();
    spi_config.frequency = Hertz(6_400_000);
    let spi = embassy_stm32::spi::Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi_config);
    
    let ws2812 = Ws2812::new(spi);
    spawner.spawn(rgb_led(ws2812)).ok();

    let i2c = I2c::new(p.I2C1, p.PA13, p.PA14, Irqs, p.DMA1_CH1, p.DMA1_CH2, Hertz(100_000), Default::default());
    let is31fl3733 = IS31FL3733::new(i2c, 0x50);
    spawner.spawn(matrix_led(is31fl3733, p.RNG)).ok();

    let driver = Driver::new(p.USB, Irqs, p.PA12, p.PA11);
    info!("HC4067 init");
    let hc4067 = Hc4067::new(p.PB12, p.PB13, p.PB15, p.PB14, p.PA15);
    info!("Virtual pins init");
    let sw_enc = Some(AnyInputPin::Gpio(Input::new(p.PC13, embassy_stm32::gpio::Pull::Up)));
    let pin_none_0 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 0xFF)));
    let pin_none_1 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 0xFF)));
    let pin_none_2 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 0xFF)));
    let pin_none_3 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 0xFF)));
    let pin0 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 0)));
    let pin1 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 1)));
    let pin2 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 2)));
    let pin3 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 3)));
    let pin4 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 4)));
    let pin5 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 5)));
    let pin6 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 6)));
    let pin7 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 7)));
    let pin8 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 8)));
    let pin9 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 9)));
    let pin10 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 10)));
    let pin11 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 11)));
    let pin12 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 12)));
    let pin13 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 13)));
    let pin14 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 14)));
    let pin15 = Some(AnyInputPin::Hc(HcPin::new(&hc4067, 15)));
    // Pin config
    info!("Configure direct pins");
    let direct_pins = config_matrix_pins_rp! {
        direct_pins: [
            [pin0, pin5, pin9, sw_enc],
            [pin1, pin6, pin10, pin15],
            [pin2, pin7, pin11, pin_none_1],
            [pin3, pin8, pin12, pin14],
        ]
    };

    // Use internal flash to emulate eeprom
    let flash = async_flash_wrapper(Flash::new_blocking(p.FLASH));

    info!("Configure Keyboard USB");
    let keyboard_usb_config = KeyboardUsbConfig {
        vid: 0x4c4b,
        pid: 0x4643,
        manufacturer: "Mechlovin's Studio,",
        product_name: "ViRust Keyboard",
        serial_number: "vial:f64c2b3c:000001",
    };

    // Keyboard config
    let rmk_config = RmkConfig {
        usb_config: keyboard_usb_config,
        vial_config: VialConfig::new(VIAL_KEYBOARD_ID, VIAL_KEYBOARD_DEF),
        ..Default::default()
    };

    // Initialize the storage and keymap
    info!("Initialising the storage and keymap");
    let mut default_keymap = keymap::get_default_keymap();
    let (keymap, storage) = initialize_keymap_and_storage(
        &mut default_keymap,
        flash,
        rmk_config.storage_config,
        rmk_config.behavior_config.clone(),
    )
    .await;

    // Initialize the matrix + keyboard
    info!("Initialising the matrix + keyboard");
    let debouncer = DefaultDebouncer::<ROW, COL>::new();
    let mut matrix = DirectPinMatrix::<_, _, ROW, COL, SIZE>::new(direct_pins, debouncer, true);
    let mut keyboard = Keyboard::new(&keymap, rmk_config.behavior_config.clone());

    // Initialize the light controller
    info!("Initialising the light controller");
    let light_controller: LightController<Output> =
        LightController::new(ControllerConfig::default().light_config);

    info!("Start the blinky task");
    spawner.spawn(blinky(led)).ok();
    // Start
    info!("Start RMK task");
    join3(
        keyboard.run(),
        run_rmk(&keymap, driver, storage, light_controller, rmk_config),
        run_devices! (
            (matrix) => EVENT_CHANNEL,
        ),
    )
    .await;
}

#[embassy_executor::task]
pub async fn blinky(mut led: Output<'static>) {
    loop {
        info!("high");
        led.set_high();
        embassy_time::Timer::after_millis(300).await;

        info!("low");
        led.set_low();
        embassy_time::Timer::after_millis(300).await;
    }
}

#[embassy_executor::task]
pub async fn matrix_led(mut led: IS31FL3733<'static, Async>, rng: RNG) {
    let mut rng = Rng::new(rng, Irqs);
    let mut leds = [0xffu8; 24];
    rng.async_fill_bytes(&mut leds).await.unwrap();
    led.initialize().await.unwrap();
    led.set_global_current_control(0xff).await.unwrap();
    led.update_leds(&leds).await.unwrap();
    led.write_brightness(0, &[0x80; 192]).await.unwrap();

    loop {
        rng.async_fill_bytes(&mut leds).await.unwrap();
        led.update_leds(&leds).await.unwrap();
        embassy_time::Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
pub async fn rgb_led(mut led: Ws2812<'static>) {
    let mut leds = [[0, 0, 0]; 5]; // 5 LEDs, default off

    loop {
        for i in 0..5 {
            leds.fill([0x00, 0x00, 0x00]);
            led.set_leds(&leds).await;
            embassy_time::Timer::after_millis(1000).await;
            info!("RGB RED");
            leds[i] = [0xFF, 0x00, 0x00]; // Red
            led.set_leds(&leds).await;
            embassy_time::Timer::after_millis(1000).await;
            info!("RGB GREEN");
            leds[i] = [0x00, 0xFF, 0x00]; // Green
            led.set_leds(&leds).await;
            embassy_time::Timer::after_millis(1000).await;
            info!("RGB BLUE");
            leds[i] = [0x00, 0x00, 0xFF]; // Blue
            led.set_leds(&leds).await;
            embassy_time::Timer::after_millis(1000).await;
        }
    }
}

#[embassy_executor::task]
pub async fn rgb_led_pwm(mut led: Ws2812Pwm<'static>) {

    loop {
        let data = [
            0xFF, 0x00, 0x00, // Red
            0x00, 0xFF, 0x00, // Green
            0x00, 0x00, 0xFF, // Blue
            0xFF, 0xFF, 0x00, // Yellow
            0x00, 0xFF, 0xFF, // Cyan
        ];
    
        led.send(&data).await;
        embassy_time::Timer::after_millis(1000).await;
    }
}