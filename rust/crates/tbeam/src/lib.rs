//! LilyGo T-Beam S3 Supreme board definition (rope unit, later both units).
//!
//! The only place that knows pin numbers and power-rail settings. Pure data
//! plus [`apply_power_profile`], generic over any `embedded-hal` I2C bus, so it
//! is host-tested; the firmware turns the numbers into ESP-IDF peripherals.
#![cfg_attr(not(test), no_std)]

pub mod pins;
mod power;

pub use power::{RAILS, RailSetting, apply_power_profile};
