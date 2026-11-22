//! SX1262 command driver.

use crate::config::{
    ChipConfig, ConfigError, LoraConfig, image_cal_args, ldro, pa_params, rf_freq_code,
    sync_word_reg, tcxo_args,
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::{Operation, SpiDevice};

mod op {
    pub const CLR_IRQ_STATUS: u8 = 0x02;
    pub const CLR_ERRORS: u8 = 0x07;
    pub const CFG_DIO_IRQ: u8 = 0x08;
    pub const WRITE_REGISTER: u8 = 0x0D;
    pub const WRITE_BUFFER: u8 = 0x0E;
    pub const GET_IRQ_STATUS: u8 = 0x12;
    pub const GET_RX_BUFFER_STATUS: u8 = 0x13;
    pub const GET_PACKET_STATUS: u8 = 0x14;
    pub const GET_ERROR: u8 = 0x17;
    pub const READ_REGISTER: u8 = 0x1D;
    pub const READ_BUFFER: u8 = 0x1E;
    pub const SET_STANDBY: u8 = 0x80;
    pub const SET_RX: u8 = 0x82;
    pub const SET_TX: u8 = 0x83;
    pub const SET_RF_FREQUENCY: u8 = 0x86;
    pub const CALIBRATE: u8 = 0x89;
    pub const SET_PACKET_TYPE: u8 = 0x8A;
    pub const SET_MODULATION_PARAMS: u8 = 0x8B;
    pub const SET_PACKET_PARAMS: u8 = 0x8C;
    pub const SET_TX_PARAMS: u8 = 0x8E;
    pub const SET_BUFFER_BASE_ADDRESS: u8 = 0x8F;
    pub const SET_PA_CONFIG: u8 = 0x95;
    pub const SET_DIO3_AS_TCXO_CTRL: u8 = 0x97;
    pub const CALIBRATE_IMAGE: u8 = 0x98;
    pub const SET_DIO2_AS_RF_SWITCH_CTRL: u8 = 0x9D;
}

const REG_SYNC_WORD: u16 = 0x0740;
const REG_TX_MODULATION: u16 = 0x0889;
const BUSY_TIMEOUT_BASE_US: u32 = 7_000;
const CALIBRATE_TYPICAL_US: u32 = 3_500;
const CALIBRATE_TIMEOUT_US: u32 = 30_000;
const BUSY_POLL_US: u32 = 10;
/// PA ramp time code 0x02 = 40 us (driver default).
const RAMP_40US: u8 = 0x02;

/// IRQ status bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct IrqFlags(pub u16);

impl IrqFlags {
    /// Transmission finished.
    pub const TX_DONE: u16 = 1 << 0;
    /// Packet received.
    pub const RX_DONE: u16 = 1 << 1;
    /// Header CRC error.
    pub const HEADER_ERR: u16 = 1 << 5;
    /// Payload CRC error.
    pub const CRC_ERR: u16 = 1 << 6;
    /// RX/TX timeout.
    pub const TIMEOUT: u16 = 1 << 9;
    /// Everything.
    pub const ALL: u16 = 0xFFFF;

    /// True if any bit of `mask` is set.
    #[must_use]
    pub const fn any(self, mask: u16) -> bool {
        self.0 & mask != 0
    }
}

/// Link quality of the last received packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacketStatus {
    /// Packet RSSI, dBm (rounded down).
    pub rssi_dbm: i16,
    /// Packet SNR, 0.25 dB.
    pub snr_qdb: i8,
}

impl PacketStatus {
    /// Decodes the `GetPacketStatus` reply bytes `[status, rssi, snr, signal_rssi]`.
    #[must_use]
    pub fn from_reply(r: [u8; 4]) -> Self {
        Self {
            rssi_dbm: -(i16::from(r[1]) + 1) / 2,
            snr_qdb: i8::from_le_bytes([r[2]]),
        }
    }
}

/// A received packet's length and quality.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RxInfo {
    /// Payload length in the caller's buffer.
    pub len: usize,
    /// RSSI/SNR.
    pub status: PacketStatus,
}

/// Driver error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<SE, PE> {
    /// SPI error.
    Spi(SE),
    /// BUSY/RESET pin error.
    Pin(PE),
    /// BUSY stayed high past the timeout.
    BusyTimeout,
    /// Unexpected chip mode after reset (raw status byte).
    BadStatus(u8),
    /// Device error flags (`GetDeviceErrors`).
    Device(u16),
    /// Invalid configuration.
    Config(ConfigError),
    /// Payload longer than 255 bytes.
    PayloadTooLong,
}

/// SX1262 driver.
#[derive(Debug)]
pub struct Sx126x<S, B, R, D> {
    spi: S,
    busy: B,
    reset: R,
    delay: D,
    busy_timeout_us: u32,
    lora: LoraConfig,
}

impl<S, B, R, D, PE> Sx126x<S, B, R, D>
where
    S: SpiDevice,
    B: InputPin<Error = PE>,
    R: OutputPin<Error = PE>,
    D: DelayNs,
{
    /// Wraps the bus and pins (no I/O).
    pub fn new(spi: S, busy: B, reset: R, delay: D) -> Self {
        Self {
            spi,
            busy,
            reset,
            delay,
            busy_timeout_us: BUSY_TIMEOUT_BASE_US,
            lora: LoraConfig::WINCHY,
        }
    }

    /// Hardware reset, chip setup, LoRa configuration and calibration.
    ///
    /// # Errors
    /// Any [`Error`]; the radio is unusable until `init` succeeds.
    pub fn init(
        &mut self,
        chip: &ChipConfig,
        lora: &LoraConfig,
    ) -> Result<(), Error<S::Error, PE>> {
        // Validate everything before touching the chip.
        if !(5..=12).contains(&lora.sf) {
            return Err(Error::Config(ConfigError::SpreadingFactor));
        }
        let image = image_cal_args(lora.freq_hz).map_err(Error::Config)?;
        let tcxo = chip
            .tcxo
            .map(tcxo_args)
            .transpose()
            .map_err(Error::Config)?;
        self.busy_timeout_us = BUSY_TIMEOUT_BASE_US + chip.tcxo.map_or(0, |t| t.start_us);
        self.lora = *lora;

        self.reset.set_low().map_err(Error::Pin)?;
        self.delay.delay_ms(1);
        self.reset.set_high().map_err(Error::Pin)?;
        self.delay.delay_ms(5);

        let status = self.read::<4>(&[op::GET_IRQ_STATUS])?[1];
        // Only the chip mode is meaningful here: it must be STDBY_RC (2) or
        // STDBY_XOSC (3). The command-status bits describe the PREVIOUS command
        // and read 5 (failure) right after reset on real boards (0xAA).
        let mode = (status >> 4) & 0x07;
        if !(mode == 2 || mode == 3) {
            return Err(Error::BadStatus(status));
        }
        if chip.dio2_rf_switch {
            self.write(&[op::SET_DIO2_AS_RF_SWITCH_CTRL, 0x01])?;
        }
        if let Some([trim, a, b, c]) = tcxo {
            self.write(&[op::SET_DIO3_AS_TCXO_CTRL, trim, a, b, c])?;
            self.delay.delay_ms(15);
            // The XOSC-start error flag from power-on (before the TCXO ran) is expected.
            self.write(&[op::CLR_ERRORS, 0x00, 0x00])?;
            self.check_device_errors()?;
        }
        let [im, il] =
            (IrqFlags::RX_DONE | IrqFlags::TX_DONE | IrqFlags::TIMEOUT | IrqFlags::CRC_ERR)
                .to_be_bytes();
        let [dm, dl] = (IrqFlags::RX_DONE | IrqFlags::TX_DONE | IrqFlags::TIMEOUT).to_be_bytes();
        self.write(&[op::CFG_DIO_IRQ, im, il, dm, dl, 0, 0, 0, 0])?;
        self.clear_irq(IrqFlags(IrqFlags::ALL))?;
        self.write(&[op::SET_PACKET_TYPE, 0x01])?; // LoRa

        let [f0, f1, f2, f3] = rf_freq_code(lora.freq_hz).to_be_bytes();
        self.write(&[op::SET_RF_FREQUENCY, f0, f1, f2, f3])?;
        let [s0, s1] = sync_word_reg(lora.sync_word).to_be_bytes();
        self.write_register(REG_SYNC_WORD, s0, s1)?;
        self.write_pa(lora.tx_power_dbm)?;
        self.write(&[
            op::SET_MODULATION_PARAMS,
            lora.sf,
            lora.bw.code(),
            lora.cr as u8,
            u8::from(ldro(lora.sf, lora.bw)),
        ])?;
        // Errata 15.1: TX modulation quality bit 2 = 0 only for BW 500 kHz.
        let [h, l] = REG_TX_MODULATION.to_be_bytes();
        let v = self.read::<5>(&[op::READ_REGISTER, h, l, 0, 0])?[4];
        let v = if lora.bw.hz() == 500_000 {
            v & !0x04
        } else {
            v | 0x04
        };
        self.write(&[op::WRITE_REGISTER, h, l, v])?;

        // Recalibrate with the TCXO running (power-on calibration ran without it).
        self.write(&[op::CALIBRATE, 0xFE])?;
        self.delay.delay_us(CALIBRATE_TYPICAL_US);
        self.wait_busy(CALIBRATE_TIMEOUT_US)?;
        self.write(&[op::CALIBRATE_IMAGE, image[0], image[1]])?;
        self.wait_busy(CALIBRATE_TIMEOUT_US)
    }

    /// Changes TX power (puts the radio in standby). Returns the applied power.
    ///
    /// # Errors
    /// SPI/BUSY errors.
    pub fn set_tx_power(&mut self, dbm: i8) -> Result<i8, Error<S::Error, PE>> {
        self.standby()?;
        let applied = dbm.clamp(-9, 22);
        self.write_pa(dbm)?;
        self.lora.tx_power_dbm = applied;
        Ok(applied)
    }

    /// Starts transmitting `payload`; DIO1 rises on `TX_DONE`.
    ///
    /// # Errors
    /// [`Error::PayloadTooLong`] or SPI/BUSY errors.
    pub fn transmit(&mut self, payload: &[u8]) -> Result<(), Error<S::Error, PE>> {
        let Ok(len) = u8::try_from(payload.len()) else {
            return Err(Error::PayloadTooLong);
        };
        self.standby()?;
        self.packet_params(len)?;
        self.write(&[op::SET_BUFFER_BASE_ADDRESS, 0x00, 0xFF])?;
        self.wait_busy(self.busy_timeout_us)?;
        self.spi
            .transaction(&mut [
                Operation::Write(&[op::WRITE_BUFFER, 0x00]),
                Operation::Write(payload),
            ])
            .map_err(Error::Spi)?;
        self.write(&[op::SET_TX, 0x00, 0x00, 0x00])
    }

    /// Enters continuous receive; DIO1 rises on `RX_DONE`.
    ///
    /// # Errors
    /// SPI/BUSY errors.
    pub fn receive_continuous(&mut self) -> Result<(), Error<S::Error, PE>> {
        self.standby()?;
        self.write(&[op::SET_BUFFER_BASE_ADDRESS, 0xFF, 0x00])?;
        self.packet_params(0xFF)?;
        self.write(&[op::SET_RX, 0xFF, 0xFF, 0xFF])
    }

    /// Reads the IRQ status.
    ///
    /// # Errors
    /// SPI/BUSY errors.
    pub fn irq_status(&mut self) -> Result<IrqFlags, Error<S::Error, PE>> {
        let r = self.read::<4>(&[op::GET_IRQ_STATUS])?;
        Ok(IrqFlags(u16::from_be_bytes([r[2], r[3]])))
    }

    /// Clears IRQ bits.
    ///
    /// # Errors
    /// SPI/BUSY errors.
    pub fn clear_irq(&mut self, flags: IrqFlags) -> Result<(), Error<S::Error, PE>> {
        let [h, l] = flags.0.to_be_bytes();
        self.write(&[op::CLR_IRQ_STATUS, h, l])
    }

    /// Copies the last received packet into `buf`.
    ///
    /// # Errors
    /// SPI/BUSY errors.
    pub fn read_packet(&mut self, buf: &mut [u8; 255]) -> Result<RxInfo, Error<S::Error, PE>> {
        let r = self.read::<4>(&[op::GET_RX_BUFFER_STATUS])?;
        let (len, ptr) = (usize::from(r[2]), r[3]);
        self.wait_busy(self.busy_timeout_us)?;
        self.spi
            .transaction(&mut [
                Operation::Write(&[op::READ_BUFFER, ptr, 0x00]),
                Operation::Read(&mut buf[..len]),
            ])
            .map_err(Error::Spi)?;
        let p = self.read::<5>(&[op::GET_PACKET_STATUS])?;
        Ok(RxInfo {
            len,
            status: PacketStatus::from_reply([p[1], p[2], p[3], p[4]]),
        })
    }

    /// Standby on the crystal oscillator (and clears pending IRQs).
    ///
    /// # Errors
    /// SPI/BUSY errors.
    pub fn standby(&mut self) -> Result<(), Error<S::Error, PE>> {
        self.write(&[op::SET_STANDBY, 0x01])?; // STDBY_XOSC
        self.clear_irq(IrqFlags(IrqFlags::ALL))
    }

    /// Releases bus, pins and delay.
    pub fn release(self) -> (S, B, R, D) {
        (self.spi, self.busy, self.reset, self.delay)
    }

    fn wait_busy(&mut self, timeout_us: u32) -> Result<(), Error<S::Error, PE>> {
        let mut waited = 0;
        while self.busy.is_high().map_err(Error::Pin)? {
            if waited >= timeout_us {
                return Err(Error::BusyTimeout);
            }
            self.delay.delay_us(BUSY_POLL_US);
            waited += BUSY_POLL_US;
        }
        Ok(())
    }

    fn write(&mut self, bytes: &[u8]) -> Result<(), Error<S::Error, PE>> {
        self.wait_busy(self.busy_timeout_us)?;
        self.spi.write(bytes).map_err(Error::Spi)
    }

    /// Full-duplex command: `cmd` followed by zeros; returns the clocked-in bytes.
    fn read<const N: usize>(&mut self, cmd: &[u8]) -> Result<[u8; N], Error<S::Error, PE>> {
        self.wait_busy(self.busy_timeout_us)?;
        let mut buf = [0u8; N];
        buf[..cmd.len()].copy_from_slice(cmd);
        self.spi.transfer_in_place(&mut buf).map_err(Error::Spi)?;
        Ok(buf)
    }

    fn write_register(&mut self, addr: u16, a: u8, b: u8) -> Result<(), Error<S::Error, PE>> {
        let [h, l] = addr.to_be_bytes();
        self.write(&[op::WRITE_REGISTER, h, l, a, b])
    }

    fn write_pa(&mut self, dbm: i8) -> Result<(), Error<S::Error, PE>> {
        let ([d, h, s, l], power) = pa_params(dbm);
        self.write(&[op::SET_PA_CONFIG, d, h, s, l])?;
        self.write(&[op::SET_TX_PARAMS, power.to_le_bytes()[0], RAMP_40US])
    }

    fn packet_params(&mut self, len: u8) -> Result<(), Error<S::Error, PE>> {
        let [ph, pl] = self.lora.preamble_len.to_be_bytes();
        // explicit header, payload length, CRC, standard IQ
        self.write(&[
            op::SET_PACKET_PARAMS,
            ph,
            pl,
            0x00,
            len,
            u8::from(self.lora.crc),
            0x00,
        ])
    }

    fn check_device_errors(&mut self) -> Result<(), Error<S::Error, PE>> {
        let r = self.read::<4>(&[op::GET_ERROR])?;
        match u16::from_be_bytes([r[2], r[3]]) {
            0 => Ok(()),
            e => Err(Error::Device(e)),
        }
    }
}

#[cfg(test)]
mod tests;
