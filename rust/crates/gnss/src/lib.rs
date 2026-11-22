//! GNSS support for both Winchy units: u-blox UBX framing and messages, a
//! minimal NMEA parser, the winch survey-in averager and the GNSS time-trust
//! decision. Pure `no_std`, no allocation: everything writes into caller
//! buffers or fixed-size internal state.
#![cfg_attr(not(test), no_std)]

pub mod baud;
pub mod detect;
pub mod nmea;
pub mod survey;
pub mod time;
pub mod ubx;
