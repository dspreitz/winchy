//! TI ADS1232 24-bit bridge ADC (rope force) over `embedded-hal` 1.0 pins.
//!
//! DRDY/DOUT falls when a conversion is ready. Waiting for that edge is the
//! caller's job (GPIO interrupt -> task notification), so this driver never
//! busy-waits; [`Ads1232::read_raw`] only clocks out a ready conversion.
//!
//! Timing: the ADS1232 enters standby if SCLK stays high for about one
//! conversion period, so the caller should run [`Ads1232::read_raw`] without
//! being preempted for long (high-priority task or critical section).
#![cfg_attr(not(test), no_std)]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};

/// PGA gain (GAIN1/GAIN0 pins).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Gain {
    /// x1.
    X1,
    /// x2.
    X2,
    /// x64.
    X64,
    /// x128.
    X128,
}

impl Gain {
    /// Pin levels (GAIN0, GAIN1).
    #[must_use]
    pub const fn pins(self) -> (bool, bool) {
        match self {
            Self::X1 => (false, false),
            Self::X2 => (true, false),
            Self::X64 => (false, true),
            Self::X128 => (true, true),
        }
    }
}

/// Driver error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// Pin error.
    Pin(E),
    /// DOUT is high: no conversion ready.
    NotReady,
}

/// The ADC's control and data pins (all sharing one error type).
#[derive(Debug)]
pub struct Pins<DOUT, SCLK, PDWN, G0, G1> {
    /// DRDY/DOUT input.
    pub dout: DOUT,
    /// Serial clock output.
    pub sclk: SCLK,
    /// Power-down output (high = running).
    pub pdwn: PDWN,
    /// GAIN0 output.
    pub gain0: G0,
    /// GAIN1 output.
    pub gain1: G1,
}

/// Half SCLK period, ns (datasheet minimum is 100 ns).
pub const HALF_PERIOD_NS: u32 = 200;

/// ADS1232 driver.
#[derive(Debug)]
pub struct Ads1232<DOUT, SCLK, PDWN, G0, G1, D> {
    pins: Pins<DOUT, SCLK, PDWN, G0, G1>,
    delay: D,
}

impl<DOUT, SCLK, PDWN, G0, G1, D, E> Ads1232<DOUT, SCLK, PDWN, G0, G1, D>
where
    DOUT: InputPin<Error = E>,
    SCLK: OutputPin<Error = E>,
    PDWN: OutputPin<Error = E>,
    G0: OutputPin<Error = E>,
    G1: OutputPin<Error = E>,
    D: DelayNs,
{
    /// Wakes the ADC (PDWN high, SCLK low) and sets the gain.
    ///
    /// # Errors
    /// Pin error.
    pub fn new(
        pins: Pins<DOUT, SCLK, PDWN, G0, G1>,
        delay: D,
        gain: Gain,
    ) -> Result<Self, Error<E>> {
        let mut adc = Self { pins, delay };
        adc.pins.sclk.set_low().map_err(Error::Pin)?;
        adc.pins.pdwn.set_high().map_err(Error::Pin)?;
        adc.set_gain(gain)?;
        Ok(adc)
    }

    /// Sets the PGA gain (the digital filter needs ~4 conversions to settle).
    ///
    /// # Errors
    /// Pin error.
    pub fn set_gain(&mut self, gain: Gain) -> Result<(), Error<E>> {
        let (g0, g1) = gain.pins();
        set(&mut self.pins.gain0, g0)?;
        set(&mut self.pins.gain1, g1)
    }

    /// True when a conversion is ready (DOUT low).
    ///
    /// # Errors
    /// Pin error.
    pub fn is_ready(&mut self) -> Result<bool, Error<E>> {
        self.pins.dout.is_low().map_err(Error::Pin)
    }

    /// Clocks out one signed 24-bit conversion (plus the 25th pulse that
    /// forces DOUT high to end the frame).
    ///
    /// # Errors
    /// [`Error::NotReady`] if DOUT is high, or pin error.
    pub fn read_raw(&mut self) -> Result<i32, Error<E>> {
        if !self.is_ready()? {
            return Err(Error::NotReady);
        }
        let mut raw: u32 = 0;
        for _ in 0..24 {
            self.pulse_high()?;
            let bit = self.pins.dout.is_high().map_err(Error::Pin)?;
            self.pulse_low()?;
            raw = (raw << 1) | u32::from(bit);
        }
        self.pulse()?; // 25th: force DOUT high, end of frame
        // Sign-extend 24 -> 32 bits: left-align, reinterpret, arithmetic shift.
        Ok(i32::from_be_bytes((raw << 8).to_be_bytes()) >> 8)
    }

    /// Starts the on-chip offset calibration (26 pulses, datasheet 7.4.1).
    /// Completion is signalled by DOUT falling again; the next conversion is settled.
    ///
    /// # Errors
    /// [`Error::NotReady`] if DOUT is high, or pin error.
    pub fn start_calibration(&mut self) -> Result<(), Error<E>> {
        if !self.is_ready()? {
            return Err(Error::NotReady);
        }
        for _ in 0..26 {
            self.pulse()?;
        }
        Ok(())
    }

    /// Enters power-down (PDWN low).
    ///
    /// # Errors
    /// Pin error.
    pub fn power_down(&mut self) -> Result<(), Error<E>> {
        self.pins.pdwn.set_low().map_err(Error::Pin)
    }

    /// Releases pins and delay.
    pub fn release(self) -> (Pins<DOUT, SCLK, PDWN, G0, G1>, D) {
        (self.pins, self.delay)
    }
}

impl<DOUT, SCLK, PDWN, G0, G1, D, E> Ads1232<DOUT, SCLK, PDWN, G0, G1, D>
where
    SCLK: OutputPin<Error = E>,
    D: DelayNs,
{
    fn pulse_high(&mut self) -> Result<(), Error<E>> {
        self.pins.sclk.set_high().map_err(Error::Pin)?;
        self.delay.delay_ns(HALF_PERIOD_NS);
        Ok(())
    }

    fn pulse_low(&mut self) -> Result<(), Error<E>> {
        self.pins.sclk.set_low().map_err(Error::Pin)?;
        self.delay.delay_ns(HALF_PERIOD_NS);
        Ok(())
    }

    fn pulse(&mut self) -> Result<(), Error<E>> {
        self.pulse_high()?;
        self.pulse_low()
    }
}

fn set<P: OutputPin>(pin: &mut P, high: bool) -> Result<(), Error<P::Error>> {
    if high { pin.set_high() } else { pin.set_low() }.map_err(Error::Pin)
}

#[cfg(test)]
mod tests;
