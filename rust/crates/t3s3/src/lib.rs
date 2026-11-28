//! LilyGo T3S3 V1.2 board definition (winch unit).
//!
//! The only place that knows the winch's pin numbers: pure data, host-tested
//! for uniqueness; the firmware turns the numbers into ESP-IDF peripherals.
//! Values are the ones the MicroPython winch ran with (`winch_app.py`).
#![cfg_attr(not(test), no_std)]

pub mod pins;
