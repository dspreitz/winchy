//! Flash data log format.
//!
//! Every measurement becomes one [`record::Record`] stamped with the
//! monotonic time of the measurement (µs since boot). Records are packed into
//! 4 KiB [`block`]s; each block header stores its base time and the offset
//! from monotonic to UTC that was valid while it was written, so every
//! sample gets an exact UTC time without spending 8 bytes on it. Blocks form
//! a ring over the flash partition ([`ring`]); a CRC rejects torn writes.
#![cfg_attr(not(test), no_std)]

pub mod block;
pub mod record;
pub mod ring;
