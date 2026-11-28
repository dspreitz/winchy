//! Winch unit application logic, free of hardware and threads.
//!
//! The winch receives the rope's telemetry, shows it to the winch driver and
//! answers each telemetry frame with at most one frame of its own (link report
//! or surveyed winch position). Every decision lives here and is host-tested;
//! the firmware only moves bytes and pixels.
#![cfg_attr(not(test), no_std)]

pub mod dashboard;
pub mod link_stats;
pub mod responder;
pub mod screen;
pub mod state;
pub mod winch_pos;
