//! LoRa link policy for the rope transmitter.
//!
//! [`adr`] picks the TX power each decision cycle from the winch's downlink
//! report and, when known, the winch<->rope distance.
#![cfg_attr(not(test), no_std)]

pub mod adr;
