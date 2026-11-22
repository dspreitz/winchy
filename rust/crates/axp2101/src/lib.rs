//! X-Powers AXP2101 power-management IC driver (`embedded-hal` 1.0 I2C).
//!
//! Register encodings follow the X-Powers datasheet as implemented by the
//! `XPowersLib` reference driver. Voltage encoding is a pure function
//! ([`voltage_code`]) so every range boundary is unit-tested; register writes
//! are read-modify-write and preserve reserved bits.
#![cfg_attr(not(test), no_std)]

mod rail;
mod regs;

pub use rail::{Rail, VoltageError, voltage_code};

use embedded_hal::i2c::I2c;

/// 7-bit I2C address.
pub const ADDRESS: u8 = 0x34;
/// Value of the chip-id register.
pub const CHIP_ID: u8 = 0x4A;

/// Driver error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// Bus error.
    I2c(E),
    /// Wrong chip id (value read).
    NotFound(u8),
    /// Requested voltage cannot be encoded for the rail.
    Voltage(VoltageError),
}

/// VBUS input voltage limit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum VbusVoltageLimit {
    /// 4.36 V.
    Mv4360 = 6,
}

/// VBUS input current limit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum VbusCurrentLimit {
    /// 1500 mA.
    Ma1500 = 4,
}

/// Power-key long-press time to power off.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PowerOffTime {
    /// 6 s.
    S6 = 1,
}

/// Power-key press time to power on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PowerOnTime {
    /// 2 s.
    S2 = 3,
}

/// Charge LED behaviour.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ChargeLed {
    /// Off.
    Off = 0,
    /// Steady on.
    On = 3,
}

/// Battery charger settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Charger {
    /// Pre-charge current code (2 = 50 mA).
    pub precharge: u8,
    /// Constant-current code (16 = 1000 mA).
    pub constant_current: u8,
    /// Termination current code (1 = 25 mA).
    pub termination: u8,
    /// Target voltage code (2 = 4.1 V).
    pub target_voltage: u8,
}

/// ADC channels to enable.
#[expect(
    clippy::struct_excessive_bools,
    reason = "one flag per hardware enable bit"
)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AdcChannels {
    /// Battery voltage.
    pub battery: bool,
    /// TS pin (battery thermistor).
    pub ts_pin: bool,
    /// VBUS voltage.
    pub vbus: bool,
    /// System voltage.
    pub system: bool,
}

/// IRQ source bits (24-bit mask over INTEN1..3).
pub mod irq {
    /// Power key long press.
    pub const PKEY_LONG: u32 = 1 << 10;
    /// Power key short press.
    pub const PKEY_SHORT: u32 = 1 << 11;
    /// Battery removed.
    pub const BAT_REMOVE: u32 = 1 << 12;
    /// Battery inserted.
    pub const BAT_INSERT: u32 = 1 << 13;
    /// VBUS removed.
    pub const VBUS_REMOVE: u32 = 1 << 14;
    /// VBUS inserted.
    pub const VBUS_INSERT: u32 = 1 << 15;
    /// Charging started.
    pub const BAT_CHG_START: u32 = 1 << 19;
    /// Charging done.
    pub const BAT_CHG_DONE: u32 = 1 << 20;
    /// PMU watchdog expired.
    pub const WDT_EXPIRE: u32 = 1 << 23;
}

/// AXP2101 driver.
#[derive(Debug)]
pub struct Axp2101<I> {
    i2c: I,
}

impl<I: I2c> Axp2101<I> {
    /// Wraps the bus (no I/O).
    pub fn new(i2c: I) -> Self {
        Self { i2c }
    }

    /// Releases the bus.
    pub fn release(self) -> I {
        self.i2c
    }

    /// Verifies the chip id.
    ///
    /// # Errors
    /// [`Error::NotFound`] on a wrong id, [`Error::I2c`] on bus failure.
    pub fn probe(&mut self) -> Result<(), Error<I::Error>> {
        let id = self.read(regs::IC_TYPE)?;
        if id == CHIP_ID {
            Ok(())
        } else {
            Err(Error::NotFound(id))
        }
    }

    /// Sets a rail's output voltage.
    ///
    /// # Errors
    /// [`Error::Voltage`] if not encodable (nothing is written), or bus error.
    pub fn set_voltage(&mut self, rail: Rail, millivolts: u16) -> Result<(), Error<I::Error>> {
        let code = voltage_code(rail, millivolts).map_err(Error::Voltage)?;
        let r = rail.regs();
        if r.keep_mask == 0 {
            self.write(r.voltage, code)
        } else {
            self.update(r.voltage, !r.keep_mask, code)
        }
    }

    /// Switches a rail on or off.
    ///
    /// # Errors
    /// Bus error.
    pub fn set_enabled(&mut self, rail: Rail, on: bool) -> Result<(), Error<I::Error>> {
        let (reg, bit) = rail.regs().enable;
        self.update(reg, 1 << bit, if on { 1 << bit } else { 0 })
    }

    /// Sets the VBUS input voltage and current limits.
    ///
    /// # Errors
    /// Bus error.
    pub fn set_vbus_limits(
        &mut self,
        voltage: VbusVoltageLimit,
        current: VbusCurrentLimit,
    ) -> Result<(), Error<I::Error>> {
        self.update(regs::INPUT_VOL_LIMIT_CTRL, 0x0F, voltage as u8)?;
        self.update(regs::INPUT_CUR_LIMIT_CTRL, 0x07, current as u8)
    }

    /// Sets the system power-down (VOFF) threshold, 2600..=3300 mV in 100 mV steps.
    ///
    /// # Errors
    /// [`Error::Voltage`] if out of range, or bus error.
    pub fn set_power_down_voltage(&mut self, millivolts: u16) -> Result<(), Error<I::Error>> {
        let code = rail::encode(&[(2600, 3300, 100, 0)], millivolts).map_err(Error::Voltage)?;
        self.update(regs::VOFF_SET, 0x07, code)
    }

    /// Power key: press times, and long press powers OFF (not restart).
    ///
    /// # Errors
    /// Bus error.
    pub fn set_power_key(
        &mut self,
        off: PowerOffTime,
        on: PowerOnTime,
    ) -> Result<(), Error<I::Error>> {
        self.update(
            regs::IRQ_OFF_ON_LEVEL_CTRL,
            0x0F,
            ((off as u8) << 2) | on as u8,
        )?;
        // PWROFF_EN: bit0 = 0 -> long press powers off (1 would restart);
        // bit1 = 1 -> long-press shutdown enabled.
        self.update(regs::PWROFF_EN, 0b11, 0b10)
    }

    /// Enables/disables ADC channels (other bits untouched).
    ///
    /// # Errors
    /// Bus error.
    pub fn set_adc_channels(&mut self, ch: AdcChannels) -> Result<(), Error<I::Error>> {
        let bits = u8::from(ch.battery)
            | u8::from(ch.ts_pin) << 1
            | u8::from(ch.vbus) << 2
            | u8::from(ch.system) << 3;
        self.update(regs::ADC_CHANNEL_CTRL, 0x0F, bits)
    }

    /// Enables battery presence detection.
    ///
    /// # Errors
    /// Bus error.
    pub fn enable_battery_detection(&mut self) -> Result<(), Error<I::Error>> {
        self.update(regs::BAT_DET_CTRL, 0x01, 0x01)
    }

    /// Sets the charge LED to manual control with the given mode.
    ///
    /// # Errors
    /// Bus error.
    pub fn set_charge_led(&mut self, mode: ChargeLed) -> Result<(), Error<I::Error>> {
        // Keep 0xC8; bits 0 and 2 select manual control; mode in bits 4..5.
        self.update(regs::CHGLED_SET_CTRL, !0xC8, 0x05 | ((mode as u8) << 4))
    }

    /// Enables the backup (button) cell charger at 2600..=3300 mV.
    ///
    /// # Errors
    /// [`Error::Voltage`] if out of range, or bus error.
    pub fn enable_backup_battery_charge(&mut self, millivolts: u16) -> Result<(), Error<I::Error>> {
        let code = rail::encode(&[(2600, 3300, 100, 0)], millivolts).map_err(Error::Voltage)?;
        self.update(regs::BTN_BAT_CHG_VOL_SET, 0x07, code)?;
        self.update(regs::CHARGE_GAUGE_WDT_CTRL, 0x04, 0x04)
    }

    /// Programs the main battery charger.
    ///
    /// # Errors
    /// Bus error.
    pub fn set_charger(&mut self, c: Charger) -> Result<(), Error<I::Error>> {
        self.update(regs::IPRECHG_SET, 0x03, c.precharge & 0x03)?;
        self.update(regs::ICC_CHG_SET, 0x1F, c.constant_current & 0x1F)?;
        self.update(regs::ITERM_CHG_SET_CTRL, 0x0F, c.termination & 0x0F)?;
        self.update(regs::CV_CHG_VOL_SET, 0x03, c.target_voltage & 0x03)
    }

    /// Writes the IRQ enable mask (bits as in [`irq`]) and clears pending status.
    ///
    /// # Errors
    /// Bus error.
    pub fn set_irq_mask(&mut self, mask: u32) -> Result<(), Error<I::Error>> {
        for (i, b) in mask.to_le_bytes()[..3].iter().enumerate() {
            self.write(regs::INTEN1 + i as u8, *b)?;
        }
        for i in 0..3 {
            self.write(regs::INTSTS1 + i, 0xFF)?;
        }
        Ok(())
    }

    /// Disables the PMU watchdog.
    ///
    /// # Errors
    /// Bus error.
    pub fn disable_watchdog(&mut self) -> Result<(), Error<I::Error>> {
        self.update(regs::CHARGE_GAUGE_WDT_CTRL, 0x01, 0x00)
    }

    /// Battery voltage in mV, `None` if no battery is connected.
    ///
    /// # Errors
    /// Bus error.
    pub fn battery_mv(&mut self) -> Result<Option<u16>, Error<I::Error>> {
        if !self.battery_connected()? {
            return Ok(None);
        }
        self.read_14(regs::ADC_VBAT_H, 0x1F).map(Some)
    }

    /// Fuel-gauge state of charge in percent, `None` without battery.
    ///
    /// # Errors
    /// Bus error.
    pub fn battery_percent(&mut self) -> Result<Option<u8>, Error<I::Error>> {
        if !self.battery_connected()? {
            return Ok(None);
        }
        self.read(regs::BAT_PERCENT_DATA).map(Some)
    }

    /// System voltage in mV.
    ///
    /// # Errors
    /// Bus error.
    pub fn system_mv(&mut self) -> Result<u16, Error<I::Error>> {
        self.read_14(regs::ADC_VSYS_H, 0x3F)
    }

    /// True while the charger is in a charging state.
    ///
    /// # Errors
    /// Bus error.
    pub fn is_charging(&mut self) -> Result<bool, Error<I::Error>> {
        Ok(self.read(regs::STATUS2)? >> 5 == 0x01)
    }
}

impl<I: I2c> Axp2101<I> {
    fn read(&mut self, reg: u8) -> Result<u8, Error<I::Error>> {
        let mut b = [0u8];
        self.i2c
            .write_read(ADDRESS, &[reg], &mut b)
            .map_err(Error::I2c)?;
        Ok(b[0])
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<I::Error>> {
        self.i2c.write(ADDRESS, &[reg, value]).map_err(Error::I2c)
    }

    /// Read-modify-write: replace the bits in `mask` with `value & mask`.
    fn update(&mut self, reg: u8, mask: u8, value: u8) -> Result<(), Error<I::Error>> {
        let old = self.read(reg)?;
        self.write(reg, (old & !mask) | (value & mask))
    }

    /// High register (masked) then low register, as two single reads.
    fn read_14(&mut self, high: u8, mask: u8) -> Result<u16, Error<I::Error>> {
        let h = self.read(high)?;
        let l = self.read(high + 1)?;
        Ok(u16::from(h & mask) << 8 | u16::from(l))
    }

    fn battery_connected(&mut self) -> Result<bool, Error<I::Error>> {
        Ok(self.read(regs::STATUS1)? & 0x08 != 0)
    }
}

#[cfg(test)]
mod tests;
