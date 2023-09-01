use bsp::hal::{
    gpio::bank0::Gpio12, gpio::bank0::Gpio13, gpio::bank0::Gpio14, gpio::bank0::Gpio15,
    gpio::bank0::Gpio7, gpio::Input, gpio::Output, gpio::Pin, gpio::PullUp, gpio::PushPull,
};
use bsp::pac::I2C0;
use rotary_encoder_embedded::{standard::StandardMode, RotaryEncoder};
use rp_pico as bsp;
use ssd1306::{mode::TerminalMode, prelude::*, Ssd1306};

pub type MetronomeState = (
    Ssd1306<
        I2CInterface<
            rp2040_hal::I2C<
                I2C0,
                (
                    rp2040_hal::gpio::Pin<
                        Gpio12,
                        rp2040_hal::gpio::Function<rp2040_hal::gpio::I2C>,
                    >,
                    rp2040_hal::gpio::Pin<
                        Gpio13,
                        rp2040_hal::gpio::Function<rp2040_hal::gpio::I2C>,
                    >,
                ),
            >,
        >,
        ssd1306::prelude::DisplaySize128x32,
        TerminalMode,
    >,
    // Jack pin
    Pin<Gpio7, Output<PushPull>>,
    // beat duration in millis
    u32,
    // Flag for pulse
    bool,
    // Alarm for encoder polling
    rp2040_hal::timer::Alarm0,
    // Alarm for next pulse
    rp2040_hal::timer::Alarm1,
    // Rotary encoder
    RotaryEncoder<StandardMode, Pin<Gpio15, Input<PullUp>>, Pin<Gpio14, Input<PullUp>>>,
);
