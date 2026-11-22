//! Output rails and their voltage encodings.

/// A switchable, programmable output rail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rail {
    /// DCDC1 (1.5-3.4 V).
    Dc1,
    /// DCDC2 (0.5-1.54 V).
    Dc2,
    /// DCDC3 (0.5-3.4 V).
    Dc3,
    /// DCDC4 (0.5-1.84 V).
    Dc4,
    /// DCDC5 (1.2 V or 1.4-3.7 V).
    Dc5,
    /// ALDO1 (0.5-3.5 V).
    Aldo1,
    /// ALDO2.
    Aldo2,
    /// ALDO3.
    Aldo3,
    /// ALDO4.
    Aldo4,
    /// BLDO1 (0.5-3.5 V).
    Bldo1,
    /// BLDO2.
    Bldo2,
    /// CPUSLDO (0.5-1.4 V).
    CpusLdo,
    /// DLDO1 (0.5-3.4 V).
    Dldo1,
    /// DLDO2.
    Dldo2,
}

/// Why a voltage cannot be encoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoltageError {
    /// Outside every range of the rail.
    OutOfRange,
    /// Inside a range but not on its step grid.
    OffStep,
}

/// Register layout of a rail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct RailRegs {
    /// Voltage register.
    pub voltage: u8,
    /// Bits of the voltage register to preserve on write.
    pub keep_mask: u8,
    /// On/off register and bit.
    pub enable: (u8, u8),
}

impl Rail {
    pub(crate) const fn regs(self) -> RailRegs {
        use crate::regs::{DC_ONOFF_DVM_CTRL as DC, LDO_ONOFF_CTRL0 as L0, LDO_ONOFF_CTRL1 as L1};
        let (voltage, keep_mask, enable) = match self {
            Self::Dc1 => (0x82, 0x00, (DC, 0)),
            Self::Dc2 => (0x83, 0x80, (DC, 1)),
            Self::Dc3 => (0x84, 0x80, (DC, 2)),
            Self::Dc4 => (0x85, 0x80, (DC, 3)),
            Self::Dc5 => (0x86, 0xE0, (DC, 4)),
            Self::Aldo1 => (0x92, 0xE0, (L0, 0)),
            Self::Aldo2 => (0x93, 0xE0, (L0, 1)),
            Self::Aldo3 => (0x94, 0xE0, (L0, 2)),
            Self::Aldo4 => (0x95, 0xE0, (L0, 3)),
            Self::Bldo1 => (0x96, 0xE0, (L0, 4)),
            Self::Bldo2 => (0x97, 0xE0, (L0, 5)),
            Self::CpusLdo => (0x98, 0xE0, (L0, 6)),
            Self::Dldo1 => (0x99, 0xE0, (L0, 7)),
            Self::Dldo2 => (0x9A, 0xE0, (L1, 0)),
        };
        RailRegs {
            voltage,
            keep_mask,
            enable,
        }
    }
}

/// Encodes `millivolts` for `rail` (the value for the voltage bits).
///
/// # Errors
/// [`VoltageError`] if the voltage is outside the rail's ranges or off-step.
pub fn voltage_code(rail: Rail, millivolts: u16) -> Result<u8, VoltageError> {
    // (min, max, step, code offset) segments, per the XPowersLib tables.
    const DC1: &[Seg] = &[(1500, 3400, 100, 0)];
    const DC2: &[Seg] = &[(500, 1200, 10, 0), (1220, 1540, 20, 71)];
    const DC3: &[Seg] = &[
        (500, 1200, 10, 0),
        (1220, 1540, 20, 71),
        (1600, 3400, 100, 88),
    ];
    const DC4: &[Seg] = &[(500, 1200, 10, 0), (1220, 1840, 20, 71)];
    const DC5: &[Seg] = &[(1200, 1200, 100, 0x19), (1400, 3700, 100, 0)];
    const LDO35: &[Seg] = &[(500, 3500, 100, 0)];
    const LDO34: &[Seg] = &[(500, 3400, 100, 0)];
    const CPUS: &[Seg] = &[(500, 1400, 50, 0)];
    let segs = match rail {
        Rail::Dc1 => DC1,
        Rail::Dc2 => DC2,
        Rail::Dc3 => DC3,
        Rail::Dc4 => DC4,
        Rail::Dc5 => DC5,
        Rail::Aldo1 | Rail::Aldo2 | Rail::Aldo3 | Rail::Aldo4 | Rail::Bldo1 | Rail::Bldo2 => LDO35,
        Rail::CpusLdo => CPUS,
        Rail::Dldo1 | Rail::Dldo2 => LDO34,
    };
    encode(segs, millivolts)
}

/// (min mV, max mV, step mV, code offset).
type Seg = (u16, u16, u16, u8);

pub(crate) fn encode(segs: &[Seg], mv: u16) -> Result<u8, VoltageError> {
    let &(min, _, step, offset) = segs
        .iter()
        .find(|&&(min, max, _, _)| (min..=max).contains(&mv))
        .ok_or(VoltageError::OutOfRange)?;
    let delta = mv - min;
    if delta % step != 0 {
        return Err(VoltageError::OffStep);
    }
    Ok(offset + (delta / step) as u8)
}

#[cfg(test)]
mod tests {
    use super::*;
    use Rail::*;

    #[test]
    fn dc1_linear() {
        assert_eq!(voltage_code(Dc1, 1500), Ok(0));
        assert_eq!(voltage_code(Dc1, 3300), Ok(18));
        assert_eq!(voltage_code(Dc1, 3400), Ok(19));
        assert_eq!(voltage_code(Dc1, 1400), Err(VoltageError::OutOfRange));
        assert_eq!(voltage_code(Dc1, 3500), Err(VoltageError::OutOfRange));
        assert_eq!(voltage_code(Dc1, 3350), Err(VoltageError::OffStep));
    }

    #[test]
    fn dc2_two_ranges() {
        assert_eq!(voltage_code(Dc2, 500), Ok(0));
        assert_eq!(voltage_code(Dc2, 1000), Ok(50));
        assert_eq!(voltage_code(Dc2, 1200), Ok(70));
        assert_eq!(voltage_code(Dc2, 1220), Ok(71));
        assert_eq!(voltage_code(Dc2, 1540), Ok(87));
        assert_eq!(voltage_code(Dc2, 1210), Err(VoltageError::OutOfRange));
        assert_eq!(voltage_code(Dc2, 1230), Err(VoltageError::OffStep));
        assert_eq!(voltage_code(Dc2, 1600), Err(VoltageError::OutOfRange));
    }

    #[test]
    fn dc3_three_ranges() {
        assert_eq!(voltage_code(Dc3, 1200), Ok(70));
        assert_eq!(voltage_code(Dc3, 1540), Ok(87));
        assert_eq!(voltage_code(Dc3, 1600), Ok(88));
        assert_eq!(voltage_code(Dc3, 3300), Ok(105));
        assert_eq!(voltage_code(Dc3, 3400), Ok(106));
        assert_eq!(voltage_code(Dc3, 1560), Err(VoltageError::OutOfRange));
        assert_eq!(voltage_code(Dc3, 3500), Err(VoltageError::OutOfRange));
    }

    #[test]
    fn dc4_extended_second_range() {
        assert_eq!(voltage_code(Dc4, 1840), Ok(102));
        assert_eq!(voltage_code(Dc4, 1860), Err(VoltageError::OutOfRange));
    }

    #[test]
    fn dc5_special_1200_and_linear() {
        assert_eq!(voltage_code(Dc5, 1200), Ok(0x19));
        assert_eq!(voltage_code(Dc5, 1400), Ok(0));
        assert_eq!(voltage_code(Dc5, 3300), Ok(19));
        assert_eq!(voltage_code(Dc5, 3700), Ok(23));
        assert_eq!(voltage_code(Dc5, 1300), Err(VoltageError::OutOfRange));
        assert_eq!(voltage_code(Dc5, 3800), Err(VoltageError::OutOfRange));
    }

    #[test]
    fn ldos() {
        for r in [Aldo1, Aldo2, Aldo3, Aldo4, Bldo1, Bldo2] {
            assert_eq!(voltage_code(r, 500), Ok(0));
            assert_eq!(voltage_code(r, 3300), Ok(28));
            assert_eq!(voltage_code(r, 3500), Ok(30));
            assert_eq!(voltage_code(r, 3600), Err(VoltageError::OutOfRange));
        }
        for r in [Dldo1, Dldo2] {
            assert_eq!(voltage_code(r, 3300), Ok(28));
            assert_eq!(voltage_code(r, 3500), Err(VoltageError::OutOfRange));
        }
        assert_eq!(voltage_code(CpusLdo, 1000), Ok(10));
        assert_eq!(voltage_code(CpusLdo, 1400), Ok(18));
        assert_eq!(voltage_code(CpusLdo, 1025), Err(VoltageError::OffStep));
        assert_eq!(voltage_code(CpusLdo, 1500), Err(VoltageError::OutOfRange));
    }

    #[test]
    fn register_map() {
        assert_eq!(
            Dc1.regs(),
            RailRegs {
                voltage: 0x82,
                keep_mask: 0x00,
                enable: (0x80, 0)
            }
        );
        assert_eq!(
            Dc2.regs(),
            RailRegs {
                voltage: 0x83,
                keep_mask: 0x80,
                enable: (0x80, 1)
            }
        );
        assert_eq!(
            Dc3.regs(),
            RailRegs {
                voltage: 0x84,
                keep_mask: 0x80,
                enable: (0x80, 2)
            }
        );
        assert_eq!(
            Dc4.regs(),
            RailRegs {
                voltage: 0x85,
                keep_mask: 0x80,
                enable: (0x80, 3)
            }
        );
        assert_eq!(
            Dc5.regs(),
            RailRegs {
                voltage: 0x86,
                keep_mask: 0xE0,
                enable: (0x80, 4)
            }
        );
        let ldos = [Aldo1, Aldo2, Aldo3, Aldo4, Bldo1, Bldo2, CpusLdo, Dldo1];
        for (i, r) in ldos.into_iter().enumerate() {
            let i = i as u8;
            assert_eq!(
                r.regs(),
                RailRegs {
                    voltage: 0x92 + i,
                    keep_mask: 0xE0,
                    enable: (0x90, i)
                }
            );
        }
        assert_eq!(
            Dldo2.regs(),
            RailRegs {
                voltage: 0x9A,
                keep_mask: 0xE0,
                enable: (0x91, 0)
            }
        );
    }
}
