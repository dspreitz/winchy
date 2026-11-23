//! Rope unit application logic, free of hardware and threads.
//!
//! The firmware runs one task per sensor rate; each task owns its driver and
//! writes its part of the shared [`state::RopeState`]. Every decision those
//! tasks take (when to warn about the battery, how to calibrate QNH, what to
//! put on the air, how loud to transmit) lives here and is host-tested.
#![cfg_attr(not(test), no_std)]

pub mod battery;
pub mod clock;
pub mod erase_gate;
pub mod force;
pub mod geometry;
pub mod last_fix;
pub mod link;
pub mod motion;
pub mod qnh;
pub mod radio;
pub mod state;
pub mod telemetry;
