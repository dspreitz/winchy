//! When the data log may erase flash ahead: every erase stops all tasks for
//! up to ~80 ms, so it is only allowed while no tow is in progress and the
//! unit has been lying still for a while (gyro quiet).

/// Bias-corrected angular rate magnitude below which the unit counts as still, dps.
pub const STILL_DPS: f32 = 3.0;
/// The unit must have been still this long, ms.
pub const STILL_FOR_MS: u64 = 5_000;

/// Gate for erasing ahead.
#[derive(Debug, Clone, Default)]
pub struct EraseGate {
    still_since: Option<u64>,
}

impl EraseGate {
    /// Closed gate.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Feeds the current gyro rate and tow state; true if erasing is allowed now.
    pub fn update(&mut self, now_ms: u64, gyro_dps: [f32; 3], idle: bool) -> bool {
        let [x, y, z] = gyro_dps;
        if !idle || libm::sqrtf(x * x + y * y + z * z) >= STILL_DPS {
            self.still_since = None;
            return false;
        }
        let since = *self.still_since.get_or_insert(now_ms);
        now_ms.saturating_sub(since) >= STILL_FOR_MS
    }
}

#[cfg(test)]
mod tests;
