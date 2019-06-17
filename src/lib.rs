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

/// Maximum number of displays connected in series supported by this lib.
const MAX_DISPLAYS: usize = 8;

/// Digits per display
const MAX_DIGITS: usize = 8;

/// Possible command register values on the display chip.
#[derive(Clone, Copy)]
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

/// Decode modes for BCD encoded input.
#[derive(Copy, Clone)]
pub enum DecodeMode
{
    NoDecode = 0x00,
    CodeBDigit0 = 0x01,
    CodeBDigits3_0 = 0x0F,
    CodeBDigits7_0 = 0xFF
}

/// 
/// Translate alphanumeric ASCII bytes into BCD
/// encoded bytes expected by the display chip.
/// 
fn bcd_byte(b: u8) -> u8 {
    match b as char {
        ' ' => 0b00001111,        // "blank"
        '-' => 0b00001010,        // - without .
        'e' => 0b00001011,        // E without .
        'E' => 0b10001011,        // E with .
        'h' => 0b00001100,        // H without .
        'H' => 0b10001100,        // H with .
        'l' => 0b00001101,        // L without .
        'L' => 0b10001101,        // L with .
        'p' => 0b00001110,        // L without .
        'P' => 0b10001110,        // L with .
        _   => b,
    }
}

/// 
/// Translate alphanumeric ASCII bytes into segment set bytes
/// 
fn ssb_byte(b: u8, dot: bool) -> u8 {
    let mut result = match b as char {
        ' ' => 0b00000000,        // "blank"
        '.' => 0b10000000,
        '-' => 0b00000001,        // -
        '0' => 0b01111110,
        '1' => 0b00110000,
        '2' => 0b01101101,
        '3' => 0b01111001,
        '4' => 0b00110011,
        '5' => 0b01011011,
        '6' => 0b01011111,
        '7' => 0b01110000,
        '8' => 0b01111111,
        '9' => 0b01111011,
        'a' | 'A' => 0b01110111,
        'b'       => 0b00011111,
        'c' | 'C' => 0b01001110,
        'd'       => 0b00111101,
        'e' | 'E' => 0b01001111,
        'f' | 'F' => 0b01000111,
        'g' | 'G' => 0b01011110,
        'h' | 'H' => 0b00110111,
        'i' | 'I' => 0b00110000,
        'j' | 'J' => 0b00111100,
        // K undoable
        'l' | 'L' => 0b00001110,
        // M undoable
        // N undoable
        'o' | 'O' => 0b01111110,
        'p' | 'P' => 0b01100111,
        'q'       => 0b01110011,
        // R undoable
        's' | 'S' => 0b01011011,
        // T undoable
        'u' | 'U' => 0b00111110,
        // V undoable
        // W undoable
        // X undoable
        // Y undoable
        // Z undoable
        _         => 0b11100101,        // ?
    };

    if dot {
        result = result | 0b10000000; // turn "." on
    }

    result
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
    devices: usize,
    buffer: [u8; MAX_DISPLAYS],
    decode_mode: DecodeMode,
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
    pub fn new(devices: usize, data: DATA, cs: CS, clk: CLK) -> Result<Self, PinError> {

        let mut num_devices = devices;
        if num_devices > MAX_DISPLAYS {
            num_devices = MAX_DISPLAYS;
        }

        let mut max7219 = MAX7219 {
            data, cs, clk, 
            devices: num_devices, 
            buffer: [0; MAX_DISPLAYS],
            decode_mode: DecodeMode::NoDecode,
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
    pub fn clear_display(&mut self, addr: usize) -> Result<(), PinError> {
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
    pub fn set_intensity(&mut self, addr: usize, intensity: u8) -> Result<(), PinError> {
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
    pub fn set_decode_mode(&mut self, addr: usize, mode: DecodeMode) -> Result<(), PinError> {
        self.decode_mode = mode; // store what we set
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
    pub fn write_data(&mut self, addr: usize, command: Command, data: u8) -> Result<(), PinError> {
        self.write_raw(addr, command as u8, data)
    }

    ///
    /// Writes byte string to the display
    /// 
    /// # Arguments
    /// 
    /// * `addrs` - list of devices over which to write the total bcd string (left to right)
    /// * `string` - the byte string to send 8 bytes long. Unknown characters result in question mark.
    /// * `dots` - u8 bit array specifying where to put dots in the string (1 = dot, 0 = not)
    ///
    /// # Errors
    /// 
    /// * `PinError` - returned in case there was an error setting a PIN on the device
    /// 
    pub fn write_str(&mut self, addr: usize, string: &[u8;MAX_DIGITS], dots: u8) -> Result<(), PinError> {
        let prev_dm = self.decode_mode;
        self.set_decode_mode(0, DecodeMode::NoDecode)?;

        let mut digit: u8 = MAX_DIGITS as u8;
        let mut dot_product: u8 = 0b10000000;
        for b in string {
            let dot = (dots & dot_product) > 0;
            dot_product = dot_product >> 1;
            self.write_raw(addr, digit, ssb_byte(*b, dot))?;

            digit = digit - 1;
        }

        self.set_decode_mode(0, prev_dm)?;

        Ok(())
    }

    ///
    /// Writes BCD encoded string to the display
    /// 
    /// # Arguments
    /// 
    /// * `addrs` - list of devices over which to write the total bcd string (left to right)
    /// * `bcd` - the bcd encoded string slice consisting of [0-9,-,E,L,H,P]
    /// where upper case input for alphabetic characters results in dot being set.
    /// Length of string is always 8 bytes, use spaces for blanking.
    ///
    /// # Errors
    /// 
    /// * `PinError` - returned in case there was an error setting a PIN on the device
    /// 
    pub fn write_bcd(&mut self, addr: usize, bcd: &[u8;MAX_DIGITS]) -> Result<(), PinError> {
        let prev_dm = self.decode_mode;
        self.set_decode_mode(0, DecodeMode::CodeBDigits7_0)?;

        let mut digit: u8 = MAX_DIGITS as u8;
        for b in bcd {
            self.write_raw(addr, digit, bcd_byte(*b))?;

            digit = digit - 1;
        }

        self.set_decode_mode(0, prev_dm)?;

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
    pub fn test(&mut self, addr: usize, is_on: bool) -> Result<(), PinError> {
        if is_on {
            self.write_data(addr, Command::DisplayTest, 0x01)
        } else {
            self.write_data(addr, Command::DisplayTest, 0x00)
        }
    }

    fn init(&mut self) -> Result<(), PinError> {
        for i in 0..self.devices {
            self.test(i, false)?; // turn testmode off
            self.write_data(i, Command::ScanLimit, 0x07)?; // set scanlimit
            self.set_decode_mode(i, DecodeMode::NoDecode)?; // direct decode
            self.clear_display(i)?; // clear all digits
        }
        self.power_off()?; // power off

        Ok(())
    }

    fn empty_buffer(&mut self) {
        self.buffer = [0; MAX_DISPLAYS];
    }

    fn write_raw(&mut self, addr: usize, header: u8, data: u8) -> Result<(), PinError> {
        let offset = addr * 2;
        let max_bytes = self.devices * 2;
        self.empty_buffer();

        self.buffer[offset] = header;
        self.buffer[offset + 1] = data;

        self.cs.set_low()?;
        for i in 0..max_bytes {
            let buffer_data = self.buffer[i];
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
