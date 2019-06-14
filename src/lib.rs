//! A platform agnostic driver to interface with the MAX7219 (LED matrix display driver)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2


#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]

extern crate embedded_hal;

use embedded_hal::digital::v2::OutputPin;

/// 
/// Possible command register values on the display chip.
/// 
pub enum Command
{
    Noop = 0x00,
    Digit0 = 0x01,
    Digit1 = 0x02,
    Digit2 = 0x03,
    Digit3 = 0x04,
    Digit4 = 0x05,
    Digit5 = 0x06,
    Digit6 = 0x07,
    Digit7 = 0x08,
    DecodeMode = 0x09,
    Intensity = 0x0A,
    ScanLimit = 0x0B,
    Power = 0x0C,
    DisplayTest = 0x0F
}

/// 
/// Decode modes for BCD encoded input.
/// 
pub enum DecodeMode
{
    NoDecode = 0x00,
    CodeBDigit0 = 0x01,
    CodeBDigits3_0 = 0x0F,
    CodeBDigits7_0 = 0xFF
}

/// 
/// Translate alphanumeric characters into BCD
/// encoded bytes expected by the display chip.
/// 
fn bcd_char(c: char) -> u8 {
    match c {
        '-' => 0b00001010, // - without .
        'e' => 0b00001011, // H without .
        'E' => 0b10001011, // H with .
        'h' => 0b00001100, // H without .
        'H' => 0b10001100, // H with .
        'l' => 0b00001101, // L without .
        'L' => 0b10001101, // L with .
        'p' => 0b00001110, // L without .
        'P' => 0b10001110, // L with .
        _ => c as u8,
    }
}

///
/// Error raised in case there was a PIN interaction
/// error during communication with the MAX7219 chip.
/// 
#[derive(Debug)]
pub struct PinError;

impl From<core::convert::Infallible> for PinError {
    fn from(_: core::convert::Infallible) -> Self {
        PinError {}
    }
}

///
/// Handles communication with the MAX7219
/// chip for segmented displays. Each display can be
/// connected in series with another and controlled via
/// a single connection.
/// 
pub struct MAX7219<DATA, CS, CLK>
{
    data: DATA,
    cs: CS,
    clk: CLK,
    devices: u8,
    buffer: [u8; 8]
}

impl<DATA, CS, CLK> MAX7219<DATA, CS, CLK>
where DATA: OutputPin, CS: OutputPin, CLK: OutputPin,
      PinError: core::convert::From<<DATA as embedded_hal::digital::v2::OutputPin>::Error>,
      PinError: core::convert::From<<CS as embedded_hal::digital::v2::OutputPin>::Error>,
      PinError: core::convert::From<<CLK as embedded_hal::digital::v2::OutputPin>::Error>,
{
    ///
    /// Returns a new MAX7219 handler for the displays
    /// Each display starts blanked, with power and test mode turned off
    /// 
    /// # Arguments
    /// 
    /// * `devices` - number of displays connected in series
    /// * `data` - the MOSI/DATA PIN previously set to Output mode
    /// * `cs` - the CS/SS PIN previously set to Output mode
    /// * `clk` - the CLK PIN previously set to Output mode
    ///
    /// # Errors
    /// 
    /// * `PinError` - returned in case there was an error setting a PIN on the device
    /// 
    pub fn new(devices: u8, data: DATA, cs: CS, clk: CLK) -> Result<Self, PinError> {

        let mut num_devices = devices;
        if num_devices > 8 {
            num_devices = 8;
        }

        let mut max7219 = MAX7219 {
            data, cs, clk, 
            devices: num_devices, 
            buffer: [0; 8]
        };

        max7219.init()?;
        Ok(max7219)
    }

    ///
    /// Powers on all connected displays
    ///
    /// # Errors
    /// 
    /// * `PinError` - returned in case there was an error setting a PIN on the device
    /// 
    pub fn power_on(&mut self) -> Result<(), PinError> {
        for i in 0..self.devices {
            self.write_data(i, Command::Power, 0x01)?;
        }

        Ok(())
    }

    ///
    /// Powers off all connected displays
    ///
    /// # Errors
    /// 
    /// * `PinError` - returned in case there was an error setting a PIN on the device
    /// 
    pub fn power_off(&mut self) -> Result<(), PinError> {
        for i in 0..self.devices {
            self.write_data(i, Command::Power, 0x00)?;
        }

        Ok(())
    }

    ///
    /// Clears display by settings all digits to empty
    /// 
    /// # Arguments
    /// 
    /// * `addr` - display to address as connected in series
    ///
    /// # Errors
    /// 
    /// * `PinError` - returned in case there was an error setting a PIN on the device
    /// 
    pub fn clear_display(&mut self, addr: u8) -> Result<(), PinError> {
        for i in 1..9 {
            self.write_raw(addr, i, 0x00)?;
        }

        Ok(())
    }

    ///
    /// Sets intensity level on the display
    /// 
    /// # Arguments
    /// 
    /// * `addr` - display to address as connected in series
    /// * `intensity` - intensity value to set to `0x00` to 0x0F`
    ///
    /// # Errors
    /// 
    /// * `PinError` - returned in case there was an error setting a PIN on the device
    /// 
    pub fn set_intensity(&mut self, addr: u8, intensity: u8) -> Result<(), PinError> {
        self.write_data(addr, Command::Intensity, intensity)
    }

    ///
    /// Sets decode mode to be used on input sent to the display chip.
    /// 
    /// # Arguments
    /// 
    /// * `addr` - display to address as connected in series
    /// * `mode` - the decode mode to set
    ///
    /// # Errors
    /// 
    /// * `PinError` - returned in case there was an error setting a PIN on the device
    /// 
    pub fn set_decode_mode(&mut self, addr: u8, mode: DecodeMode) -> Result<(), PinError> {
        self.write_data(addr, Command::DecodeMode, mode as u8)
    }

    ///
    /// Writes data to given register as described by command
    /// 
    /// # Arguments
    /// 
    /// * `addr` - display to address as connected in series
    /// * `command` - the command/register on the display to write to
    /// * `data` - the data byte value
    ///
    /// # Errors
    /// 
    /// * `PinError` - returned in case there was an error setting a PIN on the device
    /// 
    pub fn write_data(&mut self, addr: u8, command: Command, data: u8) -> Result<(), PinError> {
        self.write_raw(addr, command as u8, data)
    }

    ///
    /// Writes BCD encoded string to the display
    /// 
    /// # Arguments
    /// 
    /// * `addr` - display to address as connected in series
    /// * `bcd` - the bcd encoded string slice consisting of [0-9,-,E,L,H,P] where upper case input for alphabetic characters results in dot being set
    ///
    /// # Errors
    /// 
    /// * `PinError` - returned in case there was an error setting a PIN on the device
    /// 
    pub fn write_bcd(&mut self, addr: u8, bcd: &str) -> Result<(), PinError> {
        self.set_decode_mode(0, DecodeMode::CodeBDigits7_0)?;

        let mut digit: u8 = 8;
        for c in bcd.chars() {
            self.write_raw(addr, digit, bcd_char(c))?;

            digit = digit - 1;
            if digit == 0 {
                return Ok(())
            }
        }

        Ok(())
    }

    ///
    /// Set test mode on/off
    /// 
    /// # Arguments
    /// 
    /// * `addr` - display to address as connected in series
    /// * `is_on` - whether to turn test mode on or off
    ///
    /// # Errors
    /// 
    /// * `PinError` - returned in case there was an error setting a PIN on the device
    /// 
    pub fn test(&mut self, addr: u8, is_on: bool) -> Result<(), PinError> {
        if is_on {
            self.write_data(addr, Command::DisplayTest, 0x01)
        } else {
            self.write_data(addr, Command::DisplayTest, 0x00)
        }
    }

    fn init(&mut self) -> Result<(), PinError> {
        for i in 0..self.devices {
            self.test(i, false)?;
            self.write_data(i, Command::ScanLimit, 0x07)?;
            self.set_decode_mode(i, DecodeMode::NoDecode)?;
            self.clear_display(i)?;
        }
        self.power_off()?;

        Ok(())
    }

    fn empty_buffer(&mut self) {
        self.buffer = [0; 8];
    }

    fn write_raw(&mut self, addr: u8, header: u8, data: u8) -> Result<(), PinError> {
        let offset = addr * 2;
        let max_bytes = self.devices * 2;
        self.empty_buffer();

        self.buffer[offset as usize] = header;
        self.buffer[offset as usize + 1] = data;

        self.cs.set_low()?;
        for i in 0..max_bytes {
            let buffer_data = self.buffer[i as usize];
            self.shift_out(buffer_data)?;
        }
        self.cs.set_high()?;

        Ok(())
    }

    fn shift_out(&mut self, value: u8) -> Result<(), PinError> {
        for i in 0..8 {
            if value & (1 << (7 - i)) > 0 {
                self.data.set_high()?;
            } else {
                self.data.set_low()?;
            }

            self.clk.set_high()?;
            self.clk.set_low()?;
        }

        Ok(())
    }
}
