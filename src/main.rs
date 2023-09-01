//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::fmt::Write;
// WHAT DO THESE DO
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

mod modules;
use core::cell::RefCell;

// Board support
use rp_pico as bsp;
use bsp::{
    entry,
    hal::clocks::{init_clocks_and_plls, Clock},
    hal::pac,
    hal::sio::Sio,
    hal::timer::Timer,
    hal::timer::Alarm,
    hal::gpio::Pins,
    hal::watchdog::Watchdog,
    hal::i2c::I2C,
};
use embedded_hal::digital::v2::OutputPin;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use critical_section::Mutex;
use fugit::{MicrosDurationU32, RateExtU32};
use pac::interrupt;
use rotary_encoder_embedded::{Direction, RotaryEncoder};

use crate::modules::MetronomeState;

static mut MODULE_STATE: Mutex<RefCell<Option<MetronomeState>>> = Mutex::new(RefCell::new(None));

const ENCODER_POLL_FREQUENCY: MicrosDurationU32 = MicrosDurationU32::millis(2);
const PULSE_DURATION: MicrosDurationU32 = MicrosDurationU32::millis(5);

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut resets = pac.RESETS;
    let sio = Sio::new(pac.SIO);

    // WHY THIS
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut resets,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;

    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);
    let mut timer = Timer::new(pac.TIMER, &mut resets);

    let rotary_dt = pins.gpio15.into_pull_up_input();
    let rotary_clk = pins.gpio14.into_pull_up_input();
    let rotary_encoder = RotaryEncoder::new(rotary_dt, rotary_clk).into_standard_mode();

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_1);
    }

    critical_section::with(|cs| {
        let led = pins.gpio7.into_push_pull_output();
        let mut encoder_poll_alarm = timer.alarm_0().unwrap();
        let _ = encoder_poll_alarm.schedule(ENCODER_POLL_FREQUENCY);

        let mut pulse_alarm = timer.alarm_1().unwrap();
        let _ = pulse_alarm.schedule(MicrosDurationU32::millis(300));
        encoder_poll_alarm.enable_interrupt();
        pulse_alarm.enable_interrupt();

        unsafe {
            MODULE_STATE.borrow(cs).replace(Some((
                led,
                500,
                false,
                encoder_poll_alarm,
                pulse_alarm,
                rotary_encoder,
            )));
        }
    });

     let i2c = I2C::i2c0(
        pac.I2C0,
        pins.gpio12.into_mode(),
        pins.gpio13.into_mode(),
        150.kHz(),
        &mut resets,
        external_xtal_freq_hz.Hz(),
    );

    let interface = I2CDisplayInterface::new(i2c);
     let mut display =
        Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0).into_terminal_mode();
    display.init().unwrap();
    let _ = display.clear();
    let _ = display.write_str("this is \na message...");

    /* Endless loop */
    loop {
        // for c in 97..123 {
        //     let _ = display.write_str(unsafe { core::str::from_utf8_unchecked(&[c]) });
        // }
        // for c in 65..91 {
        //     let _ = display.write_str(unsafe { core::str::from_utf8_unchecked(&[c]) });
        // }
    }
    loop {}
}

// Handle encoder data when encoder_poll_alarm
#[interrupt]
fn TIMER_IRQ_0() {
    critical_section::with(|cs| {
        let module_state = unsafe { MODULE_STATE.borrow(cs).take() };
        if let Some((led, mut rate, pulse_flag, mut encoder_poll_alarm, pulse_alarm, mut encoder)) =
            module_state
        {
            // Clear the alarm interrupt or this interrupt service routine will keep firing
            encoder_poll_alarm.clear_interrupt();
            let _ = encoder_poll_alarm.schedule(ENCODER_POLL_FREQUENCY);

            encoder.update();
            match encoder.direction() {
                Direction::Clockwise => {
                    if rate > 20 {
                        rate -= 20
                    }
                }
                Direction::Anticlockwise => {
                    if rate < 5000 {
                        rate += 20
                    }
                }
                Direction::None => {
                    // Do nothing
                }
            }
            unsafe {
                MODULE_STATE.borrow(cs).replace_with(|_| {
                    Some((
                        led,
                        rate,
                        pulse_flag,
                        encoder_poll_alarm,
                        pulse_alarm,
                        encoder,
                    ))
                });
            }
        }
    });
}

// Handle a pulse
#[interrupt]
fn TIMER_IRQ_1() {
    critical_section::with(|cs| {
        let module_state = unsafe { MODULE_STATE.borrow(cs).take() };
        if let Some((mut led, rate, mut pulse_flag, encoder_poll_alarm, mut pulse_alarm, encoder)) =
            module_state
        {
            pulse_alarm.clear_interrupt();
            if pulse_flag {
                let _ = pulse_alarm.schedule(MicrosDurationU32::millis(rate));
                pulse_flag = false;
                led.set_low().unwrap();
            } else {
                pulse_flag = true;
                let _ = pulse_alarm.schedule(PULSE_DURATION);
                led.set_high().unwrap();
            }
            unsafe {
                MODULE_STATE.borrow(cs).replace_with(|_| {
                    Some((
                        led,
                        rate,
                        pulse_flag,
                        encoder_poll_alarm,
                        pulse_alarm,
                        encoder,
                    ))
                });
            }
        }
    });
}

// End of file
