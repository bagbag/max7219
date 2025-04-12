//! A platform agnostic driver to interface with the MAX7219 (LED matrix display driver)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2

#![deny(unsafe_code)]
#![no_std]

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiDevice;

pub mod connectors;
use connectors::*;

/// Digits per display
const MAX_DIGITS: usize = 8;

/// Possible command register values on the display chip.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Command {
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
    DisplayTest = 0x0F,
}

/// Decode modes for BCD encoded input.
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub enum DecodeMode {
    NoDecode = 0x00,
    CodeBDigit0 = 0x01,
    CodeBDigits3_0 = 0x0F,
    CodeBDigits7_0 = 0xFF,
}

///
/// Error raised in case there was an error
/// during communication with the MAX7219 chip.
///
#[derive(Debug)]
pub enum DataError {
    /// An error occurred when working with SPI
    Spi,
    /// An error occurred when working with a PIN
    Pin,
}

///
/// Handles communication with the MAX7219
/// chip for segmented displays. Each display can be
/// connected in series with another and controlled via
/// a single connection. The actual connection interface
/// is selected via constructor functions.
///
pub struct MAX7219<const D: usize, CONNECTOR> {
    connector: CONNECTOR,
    decode_mode: DecodeMode,
}

impl<const D: usize, CONNECTOR> MAX7219<D, CONNECTOR>
where
    CONNECTOR: Connector,
{
    ///
    /// Powers on all connected displays
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    pub async fn power_on(&mut self) -> Result<(), DataError> {
        for i in 0..D {
            self.write_command(i, Command::Power, 0x01).await?;
        }

        Ok(())
    }

    ///
    /// Powers off all connected displays
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    pub async fn power_off(&mut self) -> Result<(), DataError> {
        for i in 0..D {
            self.write_command(i, Command::Power, 0x00).await?;
        }

        Ok(())
    }

    ///
    /// Clears display by settings all digits to empty
    ///
    /// # Arguments
    ///
    /// * `addr` - display to address as connected in series (0 -> last)
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    pub async fn clear_display(&mut self, addr: usize) -> Result<(), DataError> {
        for i in 1..9 {
            self.write_raw_byte(addr, i, 0x00).await?;
        }

        Ok(())
    }

    ///
    /// Clears all displays by settings all digits to empty
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    pub async fn clear_all_displays(&mut self) -> Result<(), DataError> {
        let mut buffers = [[0; 2]; D];
        let buffer = buffers.as_flattened_mut();

        for digit in 1..9 {
            for display in 0..D {
                buffer[display * 2] = digit;
                buffer[display * 2 + 1] = 0x00;
            }
        }

        self.write_raw_bytes(buffer).await
    }

    ///
    /// Sets intensity level on the display
    ///
    /// # Arguments
    ///
    /// * `addr` - display to address as connected in series (0 -> last)
    /// * `intensity` - intensity value to set to `0x00` to 0x0F`
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    pub async fn set_intensity(&mut self, addr: usize, intensity: u8) -> Result<(), DataError> {
        self.write_command(addr, Command::Intensity, intensity)
            .await
    }

    ///
    /// Sets decode mode to be used on input sent to the display chip.
    ///
    /// # Arguments
    ///
    /// * `addr` - display to address as connected in series (0 -> last)
    /// * `mode` - the decode mode to set
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    pub async fn set_decode_mode(
        &mut self,
        addr: usize,
        mode: DecodeMode,
    ) -> Result<(), DataError> {
        if self.decode_mode != mode {
            self.decode_mode = mode;
            self.write_command(addr, Command::DecodeMode, mode as u8)
                .await?;
        }

        Ok(())
    }

    ///
    /// Writes byte string to the display
    ///
    /// # Arguments
    ///
    /// * `addr` - display to address as connected in series (0 -> last)
    /// * `string` - the byte string to send 8 bytes long. Unknown characters result in question mark.
    /// * `dots` - u8 bit array specifying where to put dots in the string (1 = dot, 0 = not)
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    pub async fn write_str(
        &mut self,
        addr: usize,
        string: &[u8; MAX_DIGITS],
        dots: u8,
    ) -> Result<(), DataError> {
        let prev_dm = self.decode_mode;
        self.set_decode_mode(0, DecodeMode::NoDecode).await?;

        let mut digit: u8 = MAX_DIGITS as u8;
        let mut dot_product: u8 = 0b1000_0000;
        for b in string {
            let dot = (dots & dot_product) > 0;
            dot_product >>= 1;
            self.write_raw_byte(addr, digit, ssb_byte(*b, dot)).await?;

            digit -= 1;
        }

        self.set_decode_mode(0, prev_dm).await?;

        Ok(())
    }

    ///
    /// Writes BCD encoded string to the display
    ///
    /// # Arguments
    ///
    /// * `addr` - display to address as connected in series (0 -> last)
    /// * `bcd`  - the bcd encoded string slice consisting of [0-9,-,E,L,H,P]
    ///            where upper case input for alphabetic characters results in dot being set.
    ///            Length of string is always 8 bytes, use spaces for blanking.
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    pub async fn write_bcd(
        &mut self,
        addr: usize,
        bcd: &[u8; MAX_DIGITS],
    ) -> Result<(), DataError> {
        let prev_dm = self.decode_mode;
        self.set_decode_mode(0, DecodeMode::CodeBDigits7_0).await?;

        let mut digit: u8 = MAX_DIGITS as u8;
        for b in bcd {
            self.write_raw_byte(addr, digit, bcd_byte(*b)).await?;

            digit -= 1;
        }

        self.set_decode_mode(0, prev_dm).await?;

        Ok(())
    }

    ///
    /// Writes a right justified integer with sign
    ///
    /// # Arguments
    ///
    /// * `addr` - display to address as connected in series (0 -> last)
    /// * `val` - an integer i32
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an integer over flow
    ///
    pub async fn write_integer(&mut self, addr: usize, value: i32) -> Result<(), DataError> {
        let mut buf = [0u8; 8];
        let j = base_10_bytes(value, &mut buf);
        buf = pad_left(j);
        self.write_str(addr, &buf, 0b00000000).await?;
        Ok(())
    }

    ///
    /// Writes a right justified hex formatted integer with sign
    ///
    /// # Arguments
    ///
    /// * `addr` - display to address as connected in series (0 -> last)
    /// * `val` - an integer i32
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an integer over flow
    ///
    pub async fn write_hex(&mut self, addr: usize, value: u32) -> Result<(), DataError> {
        let mut buf = [0u8; 8];
        let j = hex_bytes(value, &mut buf);
        buf = pad_left(j);
        self.write_str(addr, &buf, 0b00000000).await?;
        Ok(())
    }

    ///
    /// Writes a raw value to the display
    ///
    /// # Arguments
    ///
    /// * `addr` - display to address as connected in series (0 -> last)
    /// * `raw` - an array of raw bytes to write. Each bit represents a pixel on the display
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    pub async fn write_digits(
        &mut self,
        addr: usize,
        raw: &[u8; MAX_DIGITS],
    ) -> Result<(), DataError> {
        let prev_dm = self.decode_mode;
        self.set_decode_mode(0, DecodeMode::NoDecode).await?;

        let mut digit: u8 = 1;
        for b in raw {
            self.write_raw_byte(addr, digit, *b).await?;
            digit += 1;
        }

        self.set_decode_mode(0, prev_dm).await?;

        Ok(())
    }

    ///
    /// Set test mode on/off
    ///
    /// # Arguments
    ///
    /// * `addr` - display to address as connected in series (0 -> last)
    /// * `is_on` - whether to turn test mode on or off
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    pub async fn test(&mut self, addr: usize, is_on: bool) -> Result<(), DataError> {
        self.write_command(addr, Command::DisplayTest, is_on as u8)
            .await
    }

    // internal constructor, users should call ::from_pins or ::from_spi
    fn new(connector: CONNECTOR) -> Result<Self, DataError> {
        Ok(MAX7219 {
            connector,
            decode_mode: DecodeMode::NoDecode,
        })
    }

    pub async fn init(&mut self) -> Result<(), DataError> {
        for i in 0..D {
            self.test(i, false).await?;
            self.write_command(i, Command::ScanLimit, 0x07).await?;
            self.set_decode_mode(i, DecodeMode::NoDecode).await?;
            self.clear_display(i).await?;
        }

        self.power_off().await?;

        Ok(())
    }

    ///
    /// Writes data to given register as described by command
    ///
    /// # Arguments
    ///
    /// * `addr` - display to address as connected in series (0 -> last)
    /// * `command` - the command/register on the display to write to
    /// * `data` - the data byte value to write
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    #[inline]
    pub async fn write_command(
        &mut self,
        addr: usize,
        command: Command,
        data: u8,
    ) -> Result<(), DataError> {
        self.write_raw_byte(addr, command as u8, data).await
    }

    pub async fn write_raw_byte(
        &mut self,
        addr: usize,
        header: u8,
        data: u8,
    ) -> Result<(), DataError> {
        let offset = addr * 2;
        let mut buffers = [[0; 2]; D];
        let buffer = buffers.as_flattened_mut();

        buffer[offset] = header;
        buffer[offset + 1] = data;

        self.write_raw_bytes(buffer).await
    }

    ///
    /// Writes data to given register as described by command
    ///
    /// # Arguments
    ///
    /// * `addr` - display to address as connected in series (0 -> last)
    /// * `header` - the command/register on the display to write to as u8
    /// * `data` - the data byte value to write
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    #[inline]
    pub async fn write_raw_bytes(&mut self, buffer: &[u8]) -> Result<(), DataError> {
        self.connector.write_raw_bytes(buffer).await
    }
}

impl<const D: usize, DATA, CS, SCK> MAX7219<D, PinConnector<DATA, CS, SCK>>
where
    DATA: OutputPin,
    CS: OutputPin,
    SCK: OutputPin,
{
    ///
    /// Construct a new MAX7219 driver instance from DATA, CS and SCK pins.
    ///
    /// # Arguments
    ///
    /// * `displays` - number of displays connected in series
    /// * `data` - the MOSI/DATA PIN used to send data through to the display set to output mode
    /// * `cs` - the CS PIN used to LOAD register on the display set to output mode
    /// * `sck` - the SCK clock PIN used to drive the clock set to output mode
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    pub fn from_pins(data: DATA, cs: CS, sck: SCK) -> Result<Self, DataError> {
        MAX7219::new(PinConnector::new(data, cs, sck))
    }
}

impl<const D: usize, SPI> MAX7219<D, SpiConnector<SPI>>
where
    SPI: SpiDevice<u8>,
{
    ///
    /// Construct a new MAX7219 driver instance from pre-existing SPI in full hardware mode.
    /// The SPI will control CS (LOAD) line according to it's internal mode set.
    /// If you need the CS line to be controlled manually use MAX7219::from_spi_cs
    ///
    /// * `NOTE` - make sure the SPI is initialized in MODE_0 with max 10 Mhz frequency.
    ///
    /// # Arguments
    ///
    /// * `displays` - number of displays connected in series
    /// * `spi` - the SPI interface initialized with MOSI, MISO(unused) and CLK
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    pub fn from_spi(spi: SPI) -> Result<Self, DataError> {
        MAX7219::new(SpiConnector::new(spi))
    }
}

impl<const D: usize, SPI, CS> MAX7219<D, SpiConnectorSW<SPI, CS>>
where
    SPI: SpiDevice<u8>,
    CS: OutputPin,
{
    ///
    /// Construct a new MAX7219 driver instance from pre-existing SPI and CS pin
    /// set to output. This version of the connection uses the CS pin manually
    /// to avoid issues with how the CS mode is handled in hardware SPI implementations.
    ///
    /// * `NOTE` - make sure the SPI is initialized in MODE_0 with max 10 Mhz frequency.
    ///
    /// # Arguments
    ///
    /// * `displays` - number of displays connected in series
    /// * `spi` - the SPI interface initialized with MOSI, MISO(unused) and CLK
    /// * `cs` - the CS PIN used to LOAD register on the display set to output mode
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    pub fn from_spi_cs(spi: SPI, cs: CS) -> Result<Self, DataError> {
        MAX7219::new(SpiConnectorSW::new(spi, cs))
    }
}

///
/// Translate alphanumeric ASCII bytes into BCD
/// encoded bytes expected by the display chip.
///
fn bcd_byte(b: u8) -> u8 {
    match b as char {
        ' ' => 0b0000_1111, // "blank"
        '-' => 0b0000_1010, // - without .
        'e' => 0b0000_1011, // E without .
        'E' => 0b1000_1011, // E with .
        'h' => 0b0000_1100, // H without .
        'H' => 0b1000_1100, // H with .
        'l' => 0b0000_1101, // L without .
        'L' => 0b1000_1101, // L with .
        'p' => 0b0000_1110, // L without .
        'P' => 0b1000_1110, // L with .
        _ => b,
    }
}

///
/// Translate alphanumeric ASCII bytes into segment set bytes
///
fn ssb_byte(b: u8, dot: bool) -> u8 {
    let mut result = match b as char {
        ' ' => 0b0000_0000, // "blank"
        '.' => 0b1000_0000,
        '-' => 0b0000_0001, // -
        '_' => 0b0000_1000, // _
        '0' => 0b0111_1110,
        '1' => 0b0011_0000,
        '2' => 0b0110_1101,
        '3' => 0b0111_1001,
        '4' => 0b0011_0011,
        '5' => 0b0101_1011,
        '6' => 0b0101_1111,
        '7' => 0b0111_0000,
        '8' => 0b0111_1111,
        '9' => 0b0111_1011,
        'a' | 'A' => 0b0111_0111,
        'b' => 0b0001_1111,
        'c' | 'C' => 0b0100_1110,
        'd' => 0b0011_1101,
        'e' | 'E' => 0b0100_1111,
        'f' | 'F' => 0b0100_0111,
        'g' | 'G' => 0b0101_1110,
        'h' | 'H' => 0b0011_0111,
        'i' | 'I' => 0b0011_0000,
        'j' | 'J' => 0b0011_1100,
        // K undoable
        'l' | 'L' => 0b0000_1110,
        // M undoable
        'n' | 'N' => 0b0001_0101,
        'o' | 'O' => 0b0111_1110,
        'p' | 'P' => 0b0110_0111,
        'q' => 0b0111_0011,
        // R undoable
        's' | 'S' => 0b0101_1011,
        // T undoable
        'u' | 'U' => 0b0011_1110,
        // V undoable
        // W undoable
        // X undoable
        // Y undoable
        // Z undoable
        _ => 0b1110_0101, // ?
    };

    if dot {
        result |= 0b1000_0000; // turn "." on
    }

    result
}

///
/// Convert the integer into an integer byte Sequence
///
fn base_10_bytes(mut n: i32, buf: &mut [u8]) -> &[u8] {
    let mut sign: bool = false;
    //don't overflow the display
    if !(-9999999..99999999).contains(&n) {
        return b"Err";
    }
    if n == 0 {
        return b"0";
    }
    if n < 0 {
        n = -n;
        sign = true;
    }
    let mut i = 0;
    while n > 0 {
        buf[i] = (n % 10) as u8 + b'0';
        n /= 10;
        i += 1;
    }
    if sign {
        buf[i] = b'-';
        i += 1;
    }
    let slice = &mut buf[..i];
    slice.reverse();
    &*slice
}

//
/// Convert the integer into a hexidecimal byte Sequence
///
fn hex_bytes(mut n: u32, buf: &mut [u8]) -> &[u8] {
    // don't overflow the display ( 0xFFFFFFF)
    if n == 0 {
        return b"0";
    }
    let mut i = 0;
    while n > 0 {
        let digit = (n % 16) as u8;
        buf[i] = match digit {
            0 => b'0',
            1 => b'1',
            2 => b'2',
            3 => b'3',
            4 => b'4',
            5 => b'5',
            6 => b'6',
            7 => b'7',
            8 => b'8',
            9 => b'9',
            10 => b'a',
            11 => b'b',
            12 => b'c',
            13 => b'd',
            14 => b'e',
            15 => b'f',
            _ => b'?',
        };
        n /= 16;
        i += 1;
    }
    let slice = &mut buf[..i];
    slice.reverse();
    &*slice
}

///
/// Take a byte slice and pad the left hand side
///
fn pad_left(val: &[u8]) -> [u8; 8] {
    assert!(val.len() <= 8);
    let size: usize = 8;
    let pos: usize = val.len();
    let mut cur: usize = 1;
    let mut out: [u8; 8] = *b"        ";
    while cur <= pos {
        out[size - cur] = val[pos - cur];
        cur += 1;
    }
    out
}
