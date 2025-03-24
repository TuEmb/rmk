use embassy_stm32::mode::{Async, Mode};
use embassy_stm32::i2c::{I2c, Error};
use heapless::Vec;

#[derive(Debug)]
pub enum IS31FL3733Error {
    StateError,
    DeviceError,
    OutOfSpaceError,
}

pub const COMMAND_REGISTER: u8 = 0xfd;
pub const COMMAND_WRITE_LOCK_REGISTER: u8 = 0xfe;
pub const COMMAND_WRITE_UNLOCK: u8 = 0xc5;
pub const INTERRUPT_MASK_REGISTER: u8 = 0xf0;
pub const INTERRUPT_STATUS_REGISTER: u8 = 0xf1;

pub struct PagedRegister {
    pub page: u8,
    pub register: u8,
}

pub const LED_CONTROL_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x00,
    register: 0x00,
};
pub const LED_OPEN_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x00,
    register: 0x18,
};
pub const LED_SHORT_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x00,
    register: 0x30,
};

pub const PWM_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x01,
    register: 0x00,
};

pub const AUTO_BREATH_MODE_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x02,
    register: 0x00,
};

pub const CONFIGURATION_REGISTER: PagedRegister = PagedRegister {
    page: 0x03,
    register: 0x00,
};
pub const GCC_REGISTER: PagedRegister = PagedRegister {
    page: 0x03,
    register: 0x01,
};
pub const RESET_REGISTER: PagedRegister = PagedRegister {
    page: 0x03,
    register: 0x11,
};

pub const CONFIGURATION_SYNC_HIGH_IMPEDANCE: u8 = 0b0000_0000;
pub const CONFIGURATION_SYNC_HIGH_IMPEDANCE_ALTERNATE: u8 = 0b0110_0000;
pub const CONFIGURATION_SYNC_MASTER: u8 = 0b0010_0000;
pub const CONFIGURATION_SYNC_SLAVE: u8 = 0b0100_0000;
pub const CONFIGURATION_OSD_ENABLE: u8 = 0b0000_0100;
pub const CONFIGURATION_AUTO_BREATH_MODE_ENABLE: u8 = 0b0000_0010;
pub const CONFIGURATION_SOFTWARE_SHUTDOWN_DISABLE: u8 = 0b0000_0001;

pub const TOTAL_LED_COUNT: usize = 192;
pub struct State {
    pub page: u8,
    pub configuration_register: u8,
    pub leds: [u8; TOTAL_LED_COUNT / 8],
    pub brightness: [u8; TOTAL_LED_COUNT],
    pub global_current_control: u8,
}

impl Default for State {
    // This reflect the presumed default state after a reset
    fn default() -> Self {
        Self {
            page: 0,
            configuration_register: 0,
            leds: [0; 24],
            brightness: [0; 192],
            global_current_control: 0,
        }
    }
}

pub struct IS31FL3733<'a, M: Mode> {
    bus: I2c<'a, M>,
    address: u8,
    state: State,
    _phantom: core::marker::PhantomData<M>,
}

// General implementation
impl<'a, M: Mode> IS31FL3733<'a, M> {
    /// Create a new IS31FL3733 driver
    /// # Arguments
    /// * `i2c` - The I2C bus to use
    /// * `address` - The I2C address of the device
    ///
    /// # Returns
    /// A new IS31FL3733 driver
    pub fn new(bus: I2c<'a, M>, address: u8) -> Self {
        Self {
            bus,
            address,
            state: State::default(),
            _phantom: core::marker::PhantomData,
        }
    }

    pub fn into_inner(self) -> I2c<'a, M> {
        self.bus
    }

    pub fn inner(&self) -> &I2c<'a, M> {
        &self.bus
    }

    pub fn inner_mut(&mut self) -> &mut I2c<'a, M> {
        &mut self.bus
    }
}

impl<'a> IS31FL3733<'a, Async> {
    pub fn new_async(bus: I2c<'a, Async>, address: u8) -> Self {
        Self {
            bus,
            address,
            state: State::default(),
            _phantom: core::marker::PhantomData,
        }
    }

    async fn write(
        &mut self,
        address: u8,
        data: &[u8],
    ) -> Result<(), Error> {
        let mut buffer: Vec<u8, 200> = Vec::new();
        buffer.push(address).map_err(|_| Error::ZeroLengthTransfer)?;
        buffer.extend_from_slice(data).map_err(|_| Error::ZeroLengthTransfer)?;
        
        self.bus
            .write(
                self.address,
                &buffer
            )
            .await?;

        Ok(())
    }

    async fn read(
        &mut self,
        address: u8,
        data: &mut [u8],
    ) -> Result<(), Error> {
        self.bus.write_read(self.address, &[address], data).await?;

        Ok(())
    }

    async fn write_register(
        &mut self,
        register: u8,
        data: u8,
    ) -> Result<(), Error> {
        self.write(register, &[data]).await
    }

    async fn read_register(&mut self, register: u8) -> Result<u8, Error> {
        let mut data = [0];
        self.read(register, &mut data).await?;
        Ok(data[0])
    }

    /// Initialize the device
    ///
    /// # Returns
    /// * Ok(()) if the device was initialized successfully
    /// * Err(IS31FL3733Error::DeviceError) if the device did not respond as expected
    pub async fn initialize(&mut self) -> Result<(), IS31FL3733Error> {
        self.set_page(RESET_REGISTER.page).await?;
        let reset_result = self
            .read_register(RESET_REGISTER.register)
            .await
            .map_err(|_| IS31FL3733Error::DeviceError)?;

        if reset_result != 0x00 {
            return Err(IS31FL3733Error::DeviceError);
        }
        self.set_configuration(CONFIGURATION_SOFTWARE_SHUTDOWN_DISABLE)
            .await?;
        self.set_page(self.state.page).await?;

        Ok(())
    }

    /// Set the global current control
    ///
    /// # Arguments
    /// * `gcc` - The global current control value
    ///
    /// # Returns
    /// * Ok(()) if the global current control was set successfully
    pub async fn set_global_current_control(
        &mut self,
        gcc: u8,
    ) -> Result<(), IS31FL3733Error> {
        if self.state.global_current_control != gcc {
            self.set_page(GCC_REGISTER.page).await?;
            self.write_register(GCC_REGISTER.register, gcc)
                .await
                .map_err(|_| IS31FL3733Error::DeviceError)?;
            self.state.global_current_control = gcc;
        }
        Ok(())
    }

    /// Set the configuration register
    ///
    /// # Arguments
    /// * `configuration` - The configuration register value
    ///
    /// # Returns
    /// * Ok(()) if the configuration register was set successfully
    pub async fn set_configuration(
        &mut self,
        configuration: u8,
    ) -> Result<(), IS31FL3733Error> {
        if self.state.configuration_register != configuration {
            self.set_page(CONFIGURATION_REGISTER.page).await?;
            self.write_register(CONFIGURATION_REGISTER.register, configuration)
                .await
                .map_err(|_| IS31FL3733Error::DeviceError)?;
            self.state.configuration_register = configuration;
        }
        Ok(())
    }

    /// Unlock the page register
    ///
    /// # Returns
    /// * Ok(()) if the command register was unlocked successfully
    async fn unlock(&mut self) -> Result<(), IS31FL3733Error> {
        self.write_register(COMMAND_WRITE_LOCK_REGISTER, COMMAND_WRITE_UNLOCK)
            .await
            .map_err(|_| IS31FL3733Error::DeviceError)?;
        Ok(())
    }

    /// Set the page register, must be unlocked first using `unlock()`
    ///
    /// # Arguments
    /// * `page` - The page to set
    ///
    /// # Returns
    /// * Ok(()) if the page register was set successfully
    async fn set_page(&mut self, page: u8) -> Result<(), IS31FL3733Error> {
        if page != self.state.page {
            self.unlock().await?;
            self.write_register(COMMAND_REGISTER, page)
                .await
                .map_err(|_| IS31FL3733Error::DeviceError)?;

            self.state.page = page;
        }
        Ok(())
    }

    /// Set the LEDs state on the device. Each bit in the array represents a LED.
    /// Only the delta between the current state and the new state is written to the device.
    ///
    /// # Arguments
    /// * `leds` - The new state of the LEDs
    ///
    /// # Returns
    /// * Ok(()) if the LEDs were set successfully
    pub async fn update_leds(
        &mut self,
        leds: &[u8; TOTAL_LED_COUNT / 8],
    ) -> Result<(), IS31FL3733Error> {
        // note: I couldn't figure out a way to make diff_in_place work with async without having
        // to use boxed futures, which is not no_std compatible, so I'm reluctantly inlining here
        let current = self.state.leds;

        let byte_for_byte = current.iter().zip(leds.iter());
        let mut run_state = DiffState::Same;
        for (current, (left, right)) in byte_for_byte.enumerate() {
            match (run_state, left == right) {
                (DiffState::Same, false) => {
                    // We are starting an unequal run, preserve the current index
                    run_state = DiffState::Different(current);
                }
                (DiffState::Different(run_start), true) => {
                    // We are ending an unequal run, call the diff function
                    self.write_leds(run_start, &leds[run_start..current])
                        .await?;
                    run_state = DiffState::Same;
                }
                _ => {
                    // Run state is unchanged
                }
            }
        }

        // If we are still in a different run, call the diff function
        if let DiffState::Different(run_start) = run_state {
            self.write_leds(run_start, &leds[run_start..]).await?;
        }
        Ok(())
    }
    /// Sets the state of the LEDs on the device, starting from a
    /// specific index.
    ///
    /// # Arguments
    /// * `index` - The index of the first LED array to update
    /// * `leds` - The new brightness of the LEDs, one bit per LED
    ///
    /// # Returns
    /// * Ok(()) if the brightness was set successfully
    pub async fn write_leds(
        &mut self,
        index: usize,
        leds: &[u8],
    ) -> Result<(), IS31FL3733Error> {
        core::assert!(index + leds.len() <= TOTAL_LED_COUNT / 8);

        self.set_page(LED_CONTROL_REGISTER_BASE.page).await?;
        self.write(LED_CONTROL_REGISTER_BASE.register + index as u8, leds)
            .await
            .map_err(|_| IS31FL3733Error::DeviceError)?;

        self.state.leds[index..index + leds.len()].copy_from_slice(leds);
        Ok(())
    }

    /// Set the LEDs brightness on the device. Each byte in the array represents a LED.
    /// Only the delta between the current state and the new state is written to the device.
    ///
    /// # Arguments
    /// * `brightness` - The new state of the LEDs
    ///
    /// * Ok(()) if the LEDs were set successfully
    pub async fn update_brightness(
        &mut self,
        brightness: &[u8; TOTAL_LED_COUNT],
    ) -> Result<(), IS31FL3733Error> {
        let current = self.state.brightness;

        let byte_for_byte = current.iter().zip(brightness.iter());
        let mut run_state = DiffState::Same;
        for (current, (left, right)) in byte_for_byte.enumerate() {
            match (run_state, left == right) {
                (DiffState::Same, false) => {
                    // We are starting an unequal run, preserve the current index
                    run_state = DiffState::Different(current);
                }
                (DiffState::Different(run_start), true) => {
                    // We are ending an unequal run, call the diff function
                    self.write_brightness(
                        run_start,
                        &brightness[run_start..current],
                    )
                    .await?;
                    run_state = DiffState::Same;
                }
                _ => {
                    // Run state is unchanged
                }
            }
        }

        // If we are still in a different run, call the diff function
        if let DiffState::Different(run_start) = run_state {
            self.write_brightness(run_start, &brightness[run_start..])
                .await?;
        }

        Ok(())
    }

    /// Sets the brightness of the LEDs on the device, starting from a
    /// specific index.
    ///
    /// # Arguments
    /// * `index` - The index of the first LED to update
    /// * `brightness` - The new brightness of the LEDs, one byte per LED
    ///
    /// # Returns
    /// * Ok(()) if the brightness was set successfully
    pub async fn write_brightness(
        &mut self,
        index: usize,
        brightness: &[u8],
    ) -> Result<(), IS31FL3733Error> {
        core::assert!(index + brightness.len() <= TOTAL_LED_COUNT);

        self.set_page(PWM_REGISTER_BASE.page).await?;

        self.write(PWM_REGISTER_BASE.register + index as u8, brightness)
            .await
            .map_err(|_| IS31FL3733Error::DeviceError)?;

        self.state.brightness[index..index + brightness.len()]
            .copy_from_slice(brightness);

        Ok(())
    }
}

#[derive(Copy, Clone)]
enum DiffState {
    Same,
    Different(usize),
}
