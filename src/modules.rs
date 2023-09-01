use rotary_encoder_embedded::{standard::StandardMode, RotaryEncoder};
use rp_pico as bsp;
use bsp::hal::{
    gpio::bank0::Gpio7, gpio::bank0::Gpio14, gpio::bank0::Gpio15, gpio::Input, gpio::Output,
    gpio::Pin, gpio::PullUp, gpio::PushPull,
};

pub type MetronomeState = (
    // led pin
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
