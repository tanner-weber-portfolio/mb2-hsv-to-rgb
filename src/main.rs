/* File: main.rs
 * Author: Tanner Weber, tannerw@pdx.edu
 * Date: 11 March 2026
 */

//! Use the Microbit V2 to turn HSV to RGB for an LED
//!
//! Wiring
//!     LED Red to P8
//!     LED Green to P9
//!     LED Blue to P16
//!     LED Power to +3.3V
//!     Potentiometer Pin 1 to Gnd
//!     Potentiometer Pin 2 to P2
//!     Potentiometer Pin 3 to +3.3V


#![no_std]
#![no_main]

use core::cell::Cell;
use cortex_m_rt::entry;
use critical_section_lock_mut::LockMut;
use embedded_hal::digital::{InputPin, OutputPin};
use microbit::{
    display::blocking::Display,
    hal::{
        self, gpio,
        pac::{self, interrupt},
        pwm, saadc,
        timer::Timer,
    },
};
use panic_rtt_target as _;
use rtt_target::rprintln;

const FRAMETIME_MS: u32 = 10;
const RGB_PWM_BRIGHTNESS_STEP_MICRO_S: u32 = 100;

static LED: LockMut<RgbDisplay> = LockMut::new();

#[interrupt]
fn TIMER0() {
    rprintln!("INTERRUPT FUNC CALLED")
}

#[entry]
fn main() -> ! {
    rtt_target::rtt_init_print!();

    let board = microbit::Board::take().unwrap();
    let mut timer = Timer::new(board.TIMER0);
    let mut display = Display::new(board.display_pins);
    let mut button_a = board.buttons.button_a;
    let mut button_b = board.buttons.button_b;
    let mut leds: [[u8; 5]; 5];
    let mut state = State::Hue;
    let mut hsv = Hsv::new(0f32, 0f32, 0f32);
    let mut pot_pin = board.edge.e02.into_floating_input();
    let led_r_pin = board.edge.e08.into_push_pull_output(gpio::Level::High);
    let led_g_pin = board.edge.e09.into_push_pull_output(gpio::Level::High);
    let led_b_pin = board.edge.e16.into_push_pull_output(gpio::Level::High);
    let saadc_config = saadc::SaadcConfig::default();
    let mut saadc = saadc::Saadc::new(board.ADC, saadc_config);

    // Set up the NVIC to handle GPIO interrupts.
    unsafe { pac::NVIC::unmask(pac::Interrupt::GPIOTE) };
    pac::NVIC::unpend(pac::Interrupt::GPIOTE);

    loop {
        if button_a.is_low().unwrap() {
            rprintln!("A Pressed");
            state = state.prev();
        }
        if button_b.is_low().unwrap() {
            rprintln!("B Pressed");
            state = state.next();
        }

        match state {
            State::Hue => {
                leds = letters::get_h();
            }
            State::Saturation => {
                leds = letters::get_s();
            }
            State::Value => {
                leds = letters::get_v();
            }
        }

        // blocking read from saadc for `saadc_config.time` microseconds
        let saadc_result = saadc.read_channel(&mut pot_pin);
        rprintln!("SAADC_RESULT: {:?}", saadc_result);

        // read potentiometer
        let rgb = hsv.to_rgb();

        rprintln!("        Board {:?}", leds);
        display.show(&mut timer, leds, FRAMETIME_MS);
    }
}

struct RgbDisplay {
    frame_tick: u32,
    schedule: [u32; 3],
    next_schedule: Option<[u32; 3]>,
    timer0: Timer<pac::TIMER0>,
}

impl RgbDisplay {
    fn new<T>(
        rgb_pins: [gpio::Pin<T>; 3],
        timer0: Timer<pac::TIMER0>,
    ) -> Self {
        todo!()
    }

    fn set(&mut self, rgb: &Rgb) {
        todo!()
    }

    fn step(&mut self) {
        todo!()
    }
}

struct Hsv {
    h: f32,
    s: f32,
    v: f32,
}

impl Hsv {
    fn new(h: f32, s: f32, v: f32) -> Self {
        Hsv { h, s, v }
    }
    fn to_rgb(&self) -> Rgb {
        let rgb = Rgb {
            r: 0f32,
            g: 0f32,
            b: 0f32,
        };
        rgb
    }
}

struct Rgb {
    r: f32,
    g: f32,
    b: f32,
}

enum State {
    Hue,
    Saturation,
    Value,
}

impl State {
    fn next(&self) -> Self {
        match self {
            Self::Hue => Self::Saturation,
            Self::Saturation => Self::Value,
            Self::Value => Self::Hue,
        }
    }

    fn prev(&self) -> Self {
        match self {
            Self::Hue => Self::Value,
            Self::Saturation => Self::Hue,
            Self::Value => Self::Saturation,
        }
    }
}

mod letters {
    pub(super) fn get_h() -> [[u8; 5]; 5] {
        [
            [0, 1, 0, 1, 0],
            [0, 1, 0, 1, 0],
            [0, 1, 1, 1, 0],
            [0, 1, 0, 1, 0],
            [0, 1, 0, 1, 0],
        ]
    }

    pub(super) fn get_s() -> [[u8; 5]; 5] {
        [
            [0, 1, 1, 1, 0],
            [1, 0, 0, 1, 0],
            [0, 1, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [1, 1, 1, 0, 0],
        ]
    }

    pub(super) fn get_v() -> [[u8; 5]; 5] {
        [
            [0, 1, 0, 1, 0],
            [0, 1, 0, 1, 0],
            [0, 1, 0, 1, 0],
            [0, 1, 0, 1, 0],
            [0, 0, 1, 0, 0],
        ]
    }
}
