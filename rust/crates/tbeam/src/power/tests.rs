use super::*;
use core::convert::Infallible;
use embedded_hal::i2c::{ErrorType, Operation};

/// Simulated AXP2101 register file that logs every write.
struct RegFile {
    regs: [u8; 256],
    writes: Vec<(u8, u8)>,
}

impl RegFile {
    fn new() -> Self {
        let mut regs = [0u8; 256];
        regs[0x03] = 0x4A; // chip id
        regs[0x80] = 0x01; // DC1 on (it powers the ESP32)
        regs[0x83] = 0x80; // reserved bit set, must survive
        Self {
            regs,
            writes: vec![],
        }
    }
}

impl ErrorType for RegFile {
    type Error = Infallible;
}

impl I2c for RegFile {
    fn transaction(&mut self, addr: u8, ops: &mut [Operation<'_>]) -> Result<(), Infallible> {
        assert_eq!(addr, winchy_axp2101::ADDRESS);
        let mut ptr = 0u8;
        for op in ops {
            match op {
                Operation::Write(b) => {
                    ptr = b[0];
                    for (i, v) in b[1..].iter().enumerate() {
                        let reg = ptr.wrapping_add(i as u8);
                        self.regs[usize::from(reg)] = *v;
                        self.writes.push((reg, *v));
                    }
                }
                Operation::Read(buf) => {
                    for (i, v) in buf.iter_mut().enumerate() {
                        *v = self.regs[usize::from(ptr.wrapping_add(i as u8))];
                    }
                }
            }
        }
        Ok(())
    }
}

fn applied() -> RegFile {
    let mut pmu = Axp2101::new(RegFile::new());
    apply_power_profile(&mut pmu).unwrap();
    pmu.release()
}

#[test]
fn rail_voltages() {
    let r = applied().regs;
    assert_eq!(r[0x82], 18); // DC1 3.3 V
    assert_eq!(r[0x83], 0x80 | 0x32); // DC2 1.0 V (code 50), reserved bit kept
    assert_eq!(r[0x84], 105); // DC3 3.3 V
    assert_eq!(r[0x85], 102); // DC4 1.84 V
    assert_eq!(r[0x86], 19); // DC5 3.3 V
    assert_eq!(r[0x92..=0x97], [28; 6]); // ALDO1-4, BLDO1-2 3.3 V
    assert_eq!(r[0x98], 10); // CPUSLDO 1.0 V
    assert_eq!(r[0x99], 28); // DLDO1
    assert_eq!(r[0x9A], 28); // DLDO2
}

#[test]
fn rail_switches() {
    let r = applied().regs;
    assert_eq!(r[0x80], 0b0001_1101); // DC1 untouched (on), DC2 off, DC3-5 on
    assert_eq!(r[0x90], 0b0011_1111); // ALDO1-4 + BLDO1-2 on, CPUSLDO + DLDO1 off
    assert_eq!(r[0x91] & 0x01, 0); // DLDO2 off
}

#[test]
fn no_rail_is_switched_on_before_its_voltage_is_set() {
    let log = applied().writes;
    for s in RAILS.iter().filter(|s| s.enable == Some(true)) {
        let (vreg, en) = match s.rail {
            Rail::Dc1 => (0x82, (0x80, 0)),
            Rail::Dc2 => (0x83, (0x80, 1)),
            Rail::Dc3 => (0x84, (0x80, 2)),
            Rail::Dc4 => (0x85, (0x80, 3)),
            Rail::Dc5 => (0x86, (0x80, 4)),
            Rail::Aldo1 => (0x92, (0x90, 0)),
            Rail::Aldo2 => (0x93, (0x90, 1)),
            Rail::Aldo3 => (0x94, (0x90, 2)),
            Rail::Aldo4 => (0x95, (0x90, 3)),
            Rail::Bldo1 => (0x96, (0x90, 4)),
            Rail::Bldo2 => (0x97, (0x90, 5)),
            other => panic!("unexpected enabled rail {other:?}"),
        };
        let set_at = log
            .iter()
            .position(|&(reg, _)| reg == vreg)
            .expect("voltage written");
        let on_at = log
            .iter()
            .position(|&(reg, v)| reg == en.0 && v & (1 << en.1) != 0)
            .expect("rail enabled");
        assert!(
            set_at < on_at,
            "{:?} enabled before its voltage was set",
            s.rail
        );
    }
}

#[test]
fn input_charger_and_protection() {
    let r = applied().regs;
    assert_eq!(r[0x15] & 0x0F, 6); // VBUS 4.36 V
    assert_eq!(r[0x16] & 0x07, 4); // VBUS 1500 mA
    assert_eq!(r[0x24] & 0x07, 0); // VOFF 2.6 V
    assert_eq!(r[0x61] & 0x03, 2); // precharge 50 mA
    assert_eq!(r[0x62] & 0x1F, 16); // CC 1000 mA
    assert_eq!(r[0x63] & 0x0F, 1); // termination 25 mA
    assert_eq!(r[0x64] & 0x03, 2); // CV 4.1 V
}

#[test]
fn power_key_adc_led_backup_cell() {
    let r = applied().regs;
    assert_eq!(r[0x27] & 0x0F, (1 << 2) | 3); // off 6 s, on 2 s
    assert_eq!(r[0x22] & 0x03, 0b10); // long press = power off, enabled
    assert_eq!(r[0x30] & 0x0F, 0b1101); // VBAT, VBUS, VSYS on; TS off
    assert_eq!(r[0x68] & 0x01, 1); // battery detection
    assert_eq!(r[0x69] & 0x37, 0x05 | (3 << 4)); // charge LED manual, steady on
    assert_eq!(r[0x6A] & 0x07, 7); // backup cell 3.3 V
    assert_eq!(r[0x18] & 0x05, 0x04); // backup charge on, PMU watchdog off
}

#[test]
fn irq_mask() {
    let r = applied().regs;
    assert_eq!([r[0x40], r[0x41], r[0x42]], [0x00, 0xFC, 0x18]);
}

#[test]
fn wrong_chip_aborts_before_any_write() {
    let mut bus = RegFile::new();
    bus.regs[0x03] = 0x00;
    let mut pmu = Axp2101::new(bus);
    assert_eq!(apply_power_profile(&mut pmu), Err(Error::NotFound(0)));
    assert!(pmu.release().writes.is_empty());
}
