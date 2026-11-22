//! AXP2101 power profile (parity with the field-proven MicroPython setup).

use embedded_hal::i2c::I2c;
use winchy_axp2101::{
    AdcChannels, Axp2101, ChargeLed, Charger, Error, PowerOffTime, PowerOnTime, Rail,
    VbusCurrentLimit, VbusVoltageLimit, irq,
};

/// Voltage and switch state of one rail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RailSetting {
    /// Rail.
    pub rail: Rail,
    /// Output voltage, mV.
    pub millivolts: u16,
    /// `Some(on)` switches the rail; `None` leaves it as is (DC1 feeds the ESP32).
    pub enable: Option<bool>,
}

const fn r(rail: Rail, millivolts: u16, enable: Option<bool>) -> RailSetting {
    RailSetting {
        rail,
        millivolts,
        enable,
    }
}

/// Rail table of the T-Beam S3 Supreme.
pub const RAILS: [RailSetting; 14] = [
    r(Rail::Dc1, 3300, None),
    r(Rail::Dc2, 1000, Some(false)),
    r(Rail::Dc3, 3300, Some(true)),
    r(Rail::Dc4, 1840, Some(true)),
    r(Rail::Dc5, 3300, Some(true)),
    r(Rail::Aldo1, 3300, Some(true)),
    r(Rail::Aldo2, 3300, Some(true)),
    r(Rail::Aldo3, 3300, Some(true)),
    r(Rail::Aldo4, 3300, Some(true)),
    r(Rail::Bldo1, 3300, Some(true)),
    r(Rail::Bldo2, 3300, Some(true)),
    r(Rail::CpusLdo, 1000, Some(false)),
    r(Rail::Dldo1, 3300, Some(false)),
    r(Rail::Dldo2, 3300, Some(false)),
];

/// IRQs the firmware listens to (informational; power-off is the PMIC's own long press).
pub const IRQ_MASK: u32 = irq::BAT_INSERT
    | irq::BAT_REMOVE
    | irq::VBUS_INSERT
    | irq::VBUS_REMOVE
    | irq::PKEY_SHORT
    | irq::PKEY_LONG
    | irq::BAT_CHG_DONE
    | irq::BAT_CHG_START;

/// Probes the PMU and applies the full board power profile.
///
/// # Errors
/// Any PMU error; on error the PMU is left partially configured.
pub fn apply_power_profile<I: I2c>(pmu: &mut Axp2101<I>) -> Result<(), Error<I::Error>> {
    pmu.probe()?;
    pmu.set_vbus_limits(VbusVoltageLimit::Mv4360, VbusCurrentLimit::Ma1500)?;
    pmu.set_power_down_voltage(2600)?;
    for s in RAILS {
        // Voltage first, then switch: a rail never comes up at a stale voltage.
        pmu.set_voltage(s.rail, s.millivolts)?;
        if let Some(on) = s.enable {
            pmu.set_enabled(s.rail, on)?;
        }
    }
    pmu.set_power_key(PowerOffTime::S6, PowerOnTime::S2)?;
    pmu.set_adc_channels(AdcChannels {
        battery: true,
        ts_pin: false,
        vbus: true,
        system: true,
    })?;
    pmu.enable_battery_detection()?;
    pmu.set_charge_led(ChargeLed::On)?; // steady on = rope powered
    pmu.enable_backup_battery_charge(3300)?; // keeps GNSS ephemeris (BBR) alive
    pmu.set_charger(Charger {
        precharge: 2,         // 50 mA
        constant_current: 16, // 1000 mA
        termination: 1,       // 25 mA
        target_voltage: 2,    // 4.1 V (gentler on the cell than 4.2 V)
    })?;
    pmu.set_irq_mask(IRQ_MASK)?;
    pmu.disable_watchdog()
}

#[cfg(test)]
mod tests;
