//! Configuration of the legacy u-blox 7 (protocol 14, NEO-7M) that the winch
//! uses for now: no configuration database, so the classic CFG messages.
//!
//! The parked winch is a fixed base: the Stationary dynamic model with static
//! hold filters the fix far better than the default portable model (that one
//! let the fix jump ~520 m in a field test). SBAS (EGNOS) stays on; the NEO-7M
//! is GPS-only. Byte layouts are the ones the MicroPython winch sent.

/// UBX-CFG-NAV5 (44 bytes): apply dynamic model Stationary and static hold
/// (mask 0x0041, model 2, 50 cm/s threshold, 20 m static-hold exit distance).
#[must_use]
pub fn cfg_nav5_stationary() -> [u8; 44] {
    let mut p = [0u8; 36];
    p[0..2].copy_from_slice(&0x0041u16.to_le_bytes()); // mask: dynModel + staticHold
    p[2] = 2; // dynModel: stationary
    p[22] = 50; // staticHoldThresh, cm/s
    p[28..30].copy_from_slice(&20u16.to_le_bytes()); // staticHoldMaxDist, m
    frame(0x06, 0x24, &p)
}

/// UBX-CFG-SBAS (16 bytes): SBAS on, ranging + differential corrections.
#[must_use]
pub fn cfg_sbas() -> [u8; 16] {
    // mode: enabled; usage: range + diffCorr; maxSBAS 3; scanmode 0 (auto).
    frame(
        0x06,
        0x16,
        &[0x01, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00],
    )
}

/// UBX-CFG-CFG (21 bytes): save the current configuration to BBR + flash.
#[must_use]
pub fn cfg_save() -> [u8; 21] {
    let mut p = [0u8; 13];
    p[4..8].copy_from_slice(&0x0000_FFFFu32.to_le_bytes()); // saveMask: all sections
    p[12] = 0x17; // deviceMask: BBR, flash, EEPROM, SPI flash
    frame(0x06, 0x09, &p)
}

/// Fixed-size UBX frame (`N` = payload + 8; sizes are fixed above, so the
/// write cannot fail).
fn frame<const N: usize>(class: u8, id: u8, payload: &[u8]) -> [u8; N] {
    let mut out = [0u8; N];
    let _ = crate::ubx::write_frame(class, id, payload, &mut out);
    out
}

#[cfg(test)]
mod tests;
