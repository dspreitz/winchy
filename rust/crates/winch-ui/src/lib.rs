//! Draws a [`winchy_winch_logic::screen::Screen`] on a 128x64 monochrome
//! display, laid out like the MicroPython winch: an inverted title bar with
//! status icons, the two steering numbers in large type, small status lines.
#![cfg_attr(not(test), no_std)]

mod draw;

pub use draw::{HEIGHT, WIDTH, draw};
