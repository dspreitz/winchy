//! Sensor fusion for the rope unit. Pure `no_std` math, `f32` throughout
//! (the ESP32-S3 has a single-precision FPU; `f64` would be soft-float).
//!
//! Body frame convention: the rope runs along the device Y axis.
#![cfg_attr(not(test), no_std)]

pub mod altitude;
pub mod attitude;
pub mod gravity;
pub mod gyro_bias;
pub mod speed;
pub mod vertical;

/// A 3-axis sample or vector in the body frame.
pub type Vec3 = [f32; 3];
