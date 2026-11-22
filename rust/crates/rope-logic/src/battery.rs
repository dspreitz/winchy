//! Low-battery warning with hysteresis, only judged while idle.

/// At or below this the cell is treated as absent (USB only), mV.
pub const PRESENT_MV: u16 = 2_500;
/// Warn below this (~20 % Li-ion), mV.
pub const LOW_MV: u16 = 3_500;
/// Clear the warning above this, mV.
pub const CLEAR_MV: u16 = 3_650;

/// Low-battery latch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BatteryMonitor {
    low: bool,
}

impl BatteryMonitor {
    /// Feeds one cell reading; returns whether the warning is active.
    pub fn update(&mut self, battery_mv: Option<u16>, idle: bool) -> bool {
        match battery_mv {
            Some(mv) if idle && mv > PRESENT_MV => {
                if mv < LOW_MV {
                    self.low = true;
                } else if mv > CLEAR_MV {
                    self.low = false;
                }
            }
            _ => self.low = false,
        }
        self.low
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn low_below_threshold_while_idle() {
        let mut m = BatteryMonitor::default();
        assert!(!m.update(Some(3_700), true));
        assert!(m.update(Some(3_499), true));
    }

    #[test]
    fn hysteresis_holds_until_clear_level() {
        let mut m = BatteryMonitor::default();
        assert!(m.update(Some(3_400), true));
        assert!(m.update(Some(3_600), true), "between low and clear: stays");
        assert!(m.update(Some(3_650), true), "clear needs strictly above");
        assert!(!m.update(Some(3_651), true));
        assert!(!m.update(Some(3_600), true), "between: stays clear");
    }

    #[test]
    fn no_cell_or_usb_only_never_warns() {
        let mut m = BatteryMonitor::default();
        assert!(!m.update(None, true));
        assert!(!m.update(Some(PRESENT_MV), true));
        assert!(!m.update(Some(0), true));
    }

    #[test]
    fn warning_is_dropped_during_a_tow() {
        let mut m = BatteryMonitor::default();
        assert!(m.update(Some(3_300), true));
        assert!(!m.update(Some(3_300), false));
    }
}
