# `max7219`

> A platform agnostic driver to interface with the MAX7219 (LED display driver)

[![Build Status](https://travis-ci.org/almindor/max7219.svg?branch=master)](https://travis-ci.org/almindor/max7219)

## What works

- Powering on/off the MAX chip
- Basic commands for setting LEDs on/off.
- Chaining support (max 8 devices)

## Example

Here is a simple example for using the MAX7219 on a hifive1-revb device with e310x_hal:
```rust
#![no_std]
#![no_main]

extern crate panic_halt;

use riscv_rt::entry;
use hifive1::hal::prelude::*;
use hifive1::hal::stdout::*;
use hifive1::hal::serial::Serial;
use hifive1::hal::e310x::Peripherals;
use max7219::*;

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();

    // Configure clocks
    let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    // Configure SPI pins
    let mut gpio = p.GPIO0.split();

    let (tx, rx) = hifive1::tx_rx(
        gpio.pin17,
        gpio.pin16,
        &mut gpio.out_xor,
        &mut gpio.iof_sel,
        &mut gpio.iof_en
    );

    let data = gpio.pin3.into_output(&mut gpio.output_en, &mut gpio.drive,
                                     &mut gpio.out_xor, &mut gpio.iof_en);
    let sck = gpio.pin5.into_output(&mut gpio.output_en, &mut gpio.drive,
                                    &mut gpio.out_xor, &mut gpio.iof_en);
    let cs = gpio.pin2.into_output(&mut gpio.output_en, &mut gpio.drive,
                                   &mut gpio.out_xor, &mut gpio.iof_en);

    let mut display = MAX7219::new(1, data, cs, sck).unwrap();

    // make sure to wake the display up
    display.power_on().unwrap();
    // write given string, see function doc for permitted input in this mode
    display.write_bcd(0, "-234help").unwrap();
    // set display intensity lower
    display.set_intensity(0, 0x1).unwrap();

    loop {}
}
```

## Credits

Original work by [Maikel Wever](https://github.com/maikelwever/max7219).
Adapted to latest embedded-hal and documented by Ales Katona.

## License

Licensed under MIT license ([LICENSE](LICENSE) or http://opensource.org/licenses/MIT)

