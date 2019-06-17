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
use embedded_hal::blocking::spi::Write;

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

impl From<()> for PinError {
    fn from(_: ()) -> Self {
        PinError {}
    }
}

/// Describes the interface used to connect to the MX7219
pub trait Connector
{
    fn devices(&self) -> usize;

    ///
    /// Writes data to given register as described by command
    /// 
    /// # Arguments
    /// 
    /// * `addr` - display to address as connected in series
    /// * `command` - the command/register on the display to write to
    /// * `data` - the data byte value to write
    ///
    /// # Errors
    /// 
    /// * `PinError` - returned in case there was an error setting a PIN on the device
    /// 
    fn write_data(&mut self, addr: usize, command: Command, data: u8) -> Result<(), PinError> {
        self.write_raw(addr, command as u8, data)
    }

    ///
    /// Writes data to given register as described by command
    /// 
    /// # Arguments
    /// 
    /// * `addr` - display to address as connected in series
    /// * `header` - the command/register on the display to write to as u8
    /// * `data` - the data byte value to write
    ///
    /// # Errors
    /// 
    /// * `PinError` - returned in case there was an error setting a PIN on the device
    /// 
    fn write_raw(&mut self, addr: usize, header: u8, data: u8) -> Result<(), PinError>;
}

pub struct PinConnector<DATA, CS, SCK>
where DATA: OutputPin, CS: OutputPin, SCK: OutputPin,
{
    devices: usize,
    buffer: [u8; MAX_DISPLAYS],
    data: DATA,
    cs: CS,
    sck: SCK,
}

impl<DATA, CS, SCK> PinConnector<DATA, CS, SCK>
where DATA: OutputPin, CS: OutputPin, SCK: OutputPin,
{
    pub fn new(displays: usize, data: DATA, cs: CS, sck: SCK) -> Self {
        PinConnector {
            devices: displays,
            buffer: [0; MAX_DISPLAYS],
            data,
            cs,
            sck,
        }
    }
}

impl<DATA, CS, SCK> Connector for PinConnector<DATA, CS, SCK>
where DATA: OutputPin, CS: OutputPin, SCK: OutputPin,
      PinError: core::convert::From<<DATA as embedded_hal::digital::v2::OutputPin>::Error>,
      PinError: core::convert::From<<CS as embedded_hal::digital::v2::OutputPin>::Error>,
      PinError: core::convert::From<<SCK as embedded_hal::digital::v2::OutputPin>::Error>,
{
    fn devices(&self) -> usize {
        self.devices
    }

    fn write_raw(&mut self, addr: usize, header: u8, data: u8) -> Result<(), PinError> {
        let offset = addr * 2;
        let max_bytes = self.devices * 2;
        self.buffer = [0; MAX_DISPLAYS];

        self.buffer[offset] = header;
        self.buffer[offset + 1] = data;

        self.cs.set_low()?;
        for b in 0..max_bytes {
            let value = self.buffer[b];
            
            for i in 0..8 {
                if value & (1 << (7 - i)) > 0 {
                    self.data.set_high()?;
                } else {
                    self.data.set_low()?;
                }

                self.sck.set_high()?;
                self.sck.set_low()?;
            }

        }
        self.cs.set_high()?;

        Ok(())
    }
}

pub struct SpiConnector<SPI, CS>
where SPI: Write<u8>, CS: OutputPin,
{
    devices: usize,
    buffer: [u8; MAX_DISPLAYS],
    spi: SPI,
    cs: CS,
}

impl<SPI, CS> SpiConnector<SPI, CS>
where SPI: Write<u8>, CS: OutputPin,
{
    pub fn new(displays: usize, spi: SPI, cs: CS) -> Self {
        SpiConnector {
            devices: displays,
            buffer: [0; MAX_DISPLAYS],
            spi,
            cs,
        }
    }
}

impl<SPI, CS> Connector for SpiConnector<SPI, CS>
where SPI: Write<u8>, CS: OutputPin,
      PinError: core::convert::From<<SPI as embedded_hal::blocking::spi::Write<u8>>::Error>,
      PinError: core::convert::From<<CS as embedded_hal::digital::v2::OutputPin>::Error>,
{
    fn devices(&self) -> usize {
        self.devices
    }

    fn write_raw(&mut self, addr: usize, header: u8, data: u8) -> Result<(), PinError> {
        let offset = addr * 2;
        self.buffer = [0; MAX_DISPLAYS];

        self.buffer[offset] = header;
        self.buffer[offset + 1] = data;

        self.cs.set_low()?;
        self.spi.write(&self.buffer)?;
        self.cs.set_high()?;

        Ok(())
    }
}

///
/// Handles communication with the MAX7219
/// chip for segmented displays. Each display can be
/// connected in series with another and controlled via
/// a single connection.
///
pub struct MAX7219<CONNECTOR>
{
    c: CONNECTOR,
    decode_mode: DecodeMode,
}

impl<DATA, CS, SCK> MAX7219<PinConnector<DATA, CS, SCK>>
where DATA: OutputPin, CS: OutputPin, SCK: OutputPin,
      PinError: core::convert::From<<DATA as embedded_hal::digital::v2::OutputPin>::Error>,
      PinError: core::convert::From<<CS as embedded_hal::digital::v2::OutputPin>::Error>,
      PinError: core::convert::From<<SCK as embedded_hal::digital::v2::OutputPin>::Error>,
{
    pub fn from_pins(displays: usize, data: DATA, cs: CS, sck: SCK) -> Result<Self, PinError>
    {
        MAX7219::new(PinConnector::new(displays, data, cs, sck))
    }
}


impl<SPI, CS> MAX7219<SpiConnector<SPI, CS>>
where SPI: Write<u8>, CS: OutputPin,
      PinError: core::convert::From<()>,
      PinError: core::convert::From<<SPI as embedded_hal::blocking::spi::Write<u8>>::Error>,
      PinError: core::convert::From<<CS as embedded_hal::digital::v2::OutputPin>::Error>,
{
    pub fn from_spi(displays: usize, spi: SPI, cs: CS) -> Result<Self, PinError>
    {
        MAX7219::new(SpiConnector::new(displays, spi, cs))
    }
}

impl<CONNECTOR> MAX7219<CONNECTOR>
where CONNECTOR: Connector
{
    ///
    /// Returns a new MAX7219 handler for the displays using PINs directly.
    /// Each display starts blanked, with power and test mode turned off
    /// 
    /// # Arguments
    /// 
    /// * `connector` - the connector implementation to use to talk to the display
    ///
    /// # Errors
    /// 
    /// * `PinError` - returned in case there was an error setting a PIN on the device
    /// 
    pub fn new(connector: CONNECTOR) -> Result<Self, PinError> {
        let mut max7219 = MAX7219 {
            c: connector,
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
        for i in 0..self.c.devices() {
            self.c.write_data(i, Command::Power, 0x01)?;
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
        for i in 0..self.c.devices() {
            self.c.write_data(i, Command::Power, 0x00)?;
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
            self.c.write_raw(addr, i, 0x00)?;
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
        self.c.write_data(addr, Command::Intensity, intensity)
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
        self.c.write_data(addr, Command::DecodeMode, mode as u8)
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
            self.c.write_raw(addr, digit, ssb_byte(*b, dot))?;

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
            self.c.write_raw(addr, digit, bcd_byte(*b))?;

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
            self.c.write_data(addr, Command::DisplayTest, 0x01)
        } else {
            self.c.write_data(addr, Command::DisplayTest, 0x00)
        }
    }

    fn init(&mut self) -> Result<(), PinError> {
        for i in 0..self.c.devices() {
            self.test(i, false)?; // turn testmode off
            self.c.write_data(i, Command::ScanLimit, 0x07)?; // set scanlimit
            self.set_decode_mode(i, DecodeMode::NoDecode)?; // direct decode
            self.clear_display(i)?; // clear all digits
        }
        self.power_off()?; // power off

        Ok(())
    }
}
