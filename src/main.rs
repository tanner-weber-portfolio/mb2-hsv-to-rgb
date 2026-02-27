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
const TICKS_PER_FRAME: u32 = 100;

static LED: LockMut<LedDisplay> = LockMut::new();

#[interrupt]
fn TIMER0() {
    #[cfg(feature = "debug-output")]
    rprintln!("TIMER0 INTERRUPT FUNC CALLED");
    LED.with_lock(|f| f.step());
}

#[entry]
fn main() -> ! {
    rtt_target::rtt_init_print!();

    let board = microbit::Board::take().unwrap();
    let mut itimer = Timer::new(board.TIMER0);
    let mut dtimer = Timer::new(board.TIMER1);
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

    // Set up the NVIC to handle interrupts.
    unsafe { pac::NVIC::unmask(pac::Interrupt::TIMER0) };
    pac::NVIC::unpend(pac::Interrupt::TIMER0);
    itimer.enable_interrupt();

    LED.init(LedDisplay::new(
        [
            led_r_pin.degrade(),
            led_g_pin.degrade(),
            led_b_pin.degrade(),
        ],
        itimer,
    ));

    LED.with_lock(|f| {
        f.step();
    });

    loop {
        if button_a.is_low().unwrap() {
            #[cfg(feature = "debug-output")]
            rprintln!("A Pressed");
            state = state.get_prev();
        }
        if button_b.is_low().unwrap() {
            #[cfg(feature = "debug-output")]
            rprintln!("B Pressed");
            state = state.get_next();
        }

        // blocking read from saadc for `saadc_config.time` microseconds
        let saadc_result = saadc.read_channel(&mut pot_pin).unwrap();
        #[cfg(feature = "debug-output")]
        rprintln!("SAADC_RESULT: {:?}", saadc_result);

        let scaled_val = scale_i16(saadc_result);

        match state {
            State::Hue => {
                leds = letters::get_h();
                hsv.h = scaled_val;
            }
            State::Saturation => {
                leds = letters::get_s();
                hsv.s = scaled_val;
            }
            State::Value => {
                leds = letters::get_v();
                hsv.v = scaled_val;
            }
        }

        let rgb = hsv.to_rgb();
        LED.with_lock(|f| {
            f.set(&rgb);
        });

        #[cfg(feature = "debug-output")]
        rprintln!("        Board {:?}", leds);
        display.show(&mut dtimer, leds, FRAMETIME_MS);
    }
}

struct LedDisplay {
    /// 100 ticks per frame (100 µs), 100 frames per second (10ms).
    frame_tick: u32,
    /// What ticks the LEDs should turn off at.
    schedule: [u32; 3],
    next_schedule: Option<[u32; 3]>,
    rgb_pins: [gpio::Pin<gpio::Output<gpio::PushPull>>; 3],
    timer0: Timer<pac::TIMER0>,
}

impl LedDisplay {
    fn new<T>(
        rgb_pins: [gpio::Pin<T>; 3],
        timer0: Timer<pac::TIMER0>,
    ) -> Self {
        let [pin0, pin1, pin2] = rgb_pins;
        let pins: [gpio::Pin<gpio::Output<gpio::PushPull>>; 3] = [
            pin0.into_push_pull_output(gpio::Level::High),
            pin1.into_push_pull_output(gpio::Level::High),
            pin2.into_push_pull_output(gpio::Level::High),
        ];
        Self {
            frame_tick: 0,
            schedule: [0u32; 3],
            next_schedule: Some([0u32; 3]),
            rgb_pins: pins,
            timer0,
        }
    }

    /// Set up a new schedule, to be started next frame.
    fn set(&mut self, rgb: &Rgb) {
        self.next_schedule = Some([
            (rgb.r * 100f32 - rgb.b * 100f32) as u32,
            (rgb.g * 100f32) as u32,
            (rgb.b * 100f32 - rgb.g * 100f32) as u32,
        ]);
        self.timer0.start(64000);
    }

    /// Take the next frame update step. Called at startup
    /// and then from the timer interrupt handler.
    fn step(&mut self) {
        if self.frame_tick >= TICKS_PER_FRAME - 1 {
            self.frame_tick = 0;
            self.schedule = self.next_schedule.unwrap();
            self.timer0.start(64000);
        }

        if self.frame_tick < self.schedule[0] {
            self.rgb_pins[0].set_low();
            self.frame_tick += self.schedule[0];
        } else if self.frame_tick < self.schedule[1] {
            self.rgb_pins[1].set_low();
            self.frame_tick += self.schedule[1];
        } else {
            self.rgb_pins[2].set_low();
            self.frame_tick += self.schedule[2];
        }
    }
}

/// Hue, Saturation, and Value struct.
/// Values for each field are between 0 and 1.
struct Hsv {
    h: f32,
    s: f32,
    v: f32,
}

impl Hsv {
    fn new(h: f32, s: f32, v: f32) -> Self {
        Hsv { h, s, v }
    }

    /// Converts the 3 HSV values (ranging from 0.0 to 1.0) to RGB values
    /// (ranging from 0.0 to 1.0).
    /// Algorithm from https://en.wikipedia.org/wiki/HSL_and_HSV#HSV_to_RGB
    fn to_rgb(&self) -> Rgb {
        let mut rgb = Rgb {
            r: 0f32,
            g: 0f32,
            b: 0f32,
        };
        let hue_prime = (self.h * 360f32) / 60f32;
        let sat = self.s;
        let val = self.v;

        let chroma = val * sat;
        let x = chroma * (1f32 - (hue_prime % 2f32 - 1f32).abs());

        if hue_prime < 6f32 {
            rgb.r = chroma;
            rgb.g = 0f32;
            rgb.b = x;
        }
        if hue_prime < 5f32 {
            rgb.r = x;
            rgb.g = 0f32;
            rgb.b = chroma;
        }
        if hue_prime < 4f32 {
            rgb.r = 0f32;
            rgb.g = x;
            rgb.b = chroma;
        }
        if hue_prime < 3f32 {
            rgb.r = 0f32;
            rgb.g = chroma;
            rgb.b = x;
        }
        if hue_prime < 2f32 {
            rgb.r = x;
            rgb.g = chroma;
            rgb.b = 0f32;
        }
        if hue_prime < 1f32 {
            rgb.r = chroma;
            rgb.g = x;
            rgb.b = 0f32;
        }

        let m = val - chroma;

        rgb.r = rgb.r + m;
        rgb.g = rgb.g + m;
        rgb.b = rgb.b + m;

        Rgb {
            r: rgb.r,
            g: rgb.g,
            b: rgb.b,
        }
    }
}

/// Red, Green, and Blue colors struct.
/// Values for each field are between 0 and 1.
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
    fn get_next(&self) -> Self {
        match self {
            Self::Hue => Self::Saturation,
            Self::Saturation => Self::Value,
            Self::Value => Self::Hue,
        }
    }

    fn get_prev(&self) -> Self {
        match self {
            Self::Hue => Self::Value,
            Self::Saturation => Self::Hue,
            Self::Value => Self::Saturation,
        }
    }
}

/// H, S, and V letters represented on a 5x5 grid.
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

/// Takes an i16 number expected to be between 0 and 16384 (2^14) and scales
/// it to a value between 0.0 and 1.0.
fn scale_i16(n: i16) -> f32 {
    let n = n.clamp(0, 16000);
    (n / 16000) as f32
}
