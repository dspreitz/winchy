//! Winchy rope unit firmware. Currently a hardware bring-up diagnostic:
//! power profile, then one round of readings from every sensor.

use esp_idf_svc::hal::delay::{Ets, FreeRtos};
use esp_idf_svc::hal::peripherals::Peripherals;
use winchy_ads1232::{Ads1232, Gain};
use winchy_bmp280::Bmp280;
use winchy_board::RopeBoard;
use winchy_fusion::attitude::rope_angle_above_ground_deg;
use winchy_qmi8658::Qmi8658;
use winchy_sx126x::{ChipConfig, IrqFlags, LoraConfig, Sx126x};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("winchy-rope {} bring-up", env!("CARGO_PKG_VERSION"));

    let Some(p) = Peripherals::take().ok() else {
        log::error!("peripherals already taken");
        return;
    };
    let mut board = match RopeBoard::init(p.i2c0, p.i2c1, p.spi2, p.spi3) {
        Ok(b) => b,
        Err(e) => {
            log::error!("board init failed: {e:?}");
            return;
        }
    };

    report_pmu(&mut board);
    let RopeBoard {
        sensor_i2c,
        imu_spi,
        ads,
        lora,
        ..
    } = board;
    report_baro(sensor_i2c);
    report_imu(imu_spi);
    report_force(ads);
    radio_round_trip(lora);
    log::info!("bring-up done");
}

fn report_pmu(board: &mut RopeBoard) {
    let pmu = &mut board.pmu;
    log::info!(
        "PMU: vsys {:?} mV, battery {:?} mV ({:?} %), charging {:?}",
        pmu.system_mv(),
        pmu.battery_mv(),
        pmu.battery_percent(),
        pmu.is_charging()
    );
}

fn report_baro(i2c: esp_idf_svc::hal::i2c::I2cDriver<'static>) {
    let mut bmp = match Bmp280::new(
        i2c,
        winchy_bmp280::ADDRESS,
        winchy_bmp280::Config::default(),
    ) {
        Ok(b) => b,
        Err(e) => {
            log::error!("BMP280: {e:?}");
            return;
        }
    };
    for _ in 0..3 {
        if let Err(e) = bmp.start_forced() {
            log::error!("BMP280 start: {e:?}");
            return;
        }
        FreeRtos::delay_ms(5);
        while bmp.is_measuring().unwrap_or(false) {
            FreeRtos::delay_ms(5);
        }
        match bmp.read() {
            Ok(m) => log::info!(
                "BMP280: {:.2} hPa, {:.2} degC, plausible {}",
                m.pressure_hpa(),
                f64::from(m.temperature_cdeg) / 100.0,
                m.is_plausible()
            ),
            Err(e) => log::error!("BMP280 read: {e:?}"),
        }
    }
}

fn report_imu(
    spi: esp_idf_svc::hal::spi::SpiDeviceDriver<'static, esp_idf_svc::hal::spi::SpiDriver<'static>>,
) {
    let mut imu = match Qmi8658::new(spi, &mut FreeRtos) {
        Ok(i) => i,
        Err(e) => {
            log::error!("QMI8658: {e:?}");
            return;
        }
    };
    FreeRtos::delay_ms(50);
    for _ in 0..5 {
        match imu.read_raw() {
            Ok(s) => {
                let a = s.accel_g();
                let norm = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
                log::info!(
                    "QMI8658: raw {:?}/{:?} accel {a:.3?} g |a| {norm:.3} gyro {:.2?} dps rope {:.1} deg",
                    s.accel,
                    s.gyro,
                    s.gyro_dps(),
                    rope_angle_above_ground_deg(a)
                );
            }
            Err(e) => log::error!("QMI8658 read: {e:?}"),
        }
        FreeRtos::delay_ms(20);
    }
    log::info!("QMI8658: temperature {:?} degC", imu.temperature_c());
}

fn report_force(pins: winchy_board::AdsPins) {
    let mut adc = match Ads1232::new(pins, Ets, Gain::X128) {
        Ok(a) => a,
        Err(e) => {
            log::error!("ADS1232: {e:?}");
            return;
        }
    };
    for _ in 0..5 {
        let mut waited = 0;
        while !adc.is_ready().unwrap_or(false) && waited < 500 {
            FreeRtos::delay_ms(1);
            waited += 1;
        }
        if waited >= 500 {
            log::warn!("ADS1232: no DRDY within 500 ms (load cell missing?)");
            return;
        }
        // Keep the 25 SCLK pulses unbroken (standby if SCLK stays high).
        let r = esp_idf_svc::hal::interrupt::free(|| adc.read_raw());
        log::info!("ADS1232: raw {r:?} (after {waited} ms)");
    }
}

/// Legacy (MicroPython protocol v6) TELEMETRY frame with `REQUEST_REPORT`, so
/// the unmodified winch answers with a v6 `LINK_REPORT`. Test-only.
fn legacy_v6_telemetry(seq: u16) -> [u8; 17] {
    let mut f = [0u8; 17];
    f[0] = 6; // version
    f[1] = 1; // TELEMETRY
    f[2..4].copy_from_slice(&seq.to_le_bytes());
    // phase 0, force 0, angle 0, altitude 0, battery 0
    f[13] = 0x08 | 0x01; // flags: REQUEST_REPORT | FORCE_UNCALIBRATED
    f[14] = 255; // battery percent unknown
    f
}

fn radio_round_trip(p: winchy_board::LoraPins) {
    let mut radio = Sx126x::new(p.spi, p.busy, p.reset, Ets);
    if let Err(e) = radio.init(&ChipConfig::LILYGO, &LoraConfig::WINCHY) {
        log::error!("SX1262 init: {e:?}");
        return;
    }
    log::info!("SX1262: initialised (869.525 MHz SF7 BW250 CR4/8 sync 0x12)");
    let mut dio1 = p.dio1;
    let mut buf = [0u8; 255];
    let (mut sent, mut replies) = (0u16, 0u32);
    for seq in 0..6u16 {
        if let Err(e) = radio.transmit(&legacy_v6_telemetry(seq)) {
            log::error!("SX1262 tx: {e:?}");
            return;
        }
        if !wait_irq(&mut radio, &mut dio1, IrqFlags::TX_DONE, 500) {
            log::warn!("SX1262: no TX_DONE");
            continue;
        }
        sent += 1;
        let _ = radio.receive_continuous();
        // Listen ~1.5 s for the winch's answer.
        for _ in 0..300 {
            FreeRtos::delay_ms(5);
            if dio1.is_low() {
                continue;
            }
            let Ok(flags) = radio.irq_status() else {
                continue;
            };
            let _ = radio.clear_irq(flags);
            if !flags.any(IrqFlags::RX_DONE) {
                continue;
            }
            match radio.read_packet(&mut buf) {
                Ok(info) => {
                    let f = &buf[..info.len];
                    let crc_ok = !flags.any(IrqFlags::CRC_ERR);
                    log::info!(
                        "SX1262 rx: {} B, rssi {} dBm, snr {} qdB, crc_ok {crc_ok}, bytes {:02x?}",
                        info.len,
                        info.status.rssi_dbm,
                        info.status.snr_qdb,
                        f
                    );
                    if crc_ok && f.len() == 8 && f[0] == 6 && f[1] == 5 {
                        replies += 1;
                        log::info!(
                            "  winch LINK_REPORT: our frame arrived with rssi {} dBm, snr {} qdB, loss {} %",
                            i16::from_le_bytes([f[4], f[5]]),
                            i8::from_le_bytes([f[6]]),
                            f[7]
                        );
                    }
                }
                Err(e) => log::error!("SX1262 read: {e:?}"),
            }
            break;
        }
    }
    log::info!("SX1262: sent {sent}, winch replies {replies}");
}

fn wait_irq<S, B, R, D, PE>(
    radio: &mut Sx126x<S, B, R, D>,
    dio1: &mut esp_idf_svc::hal::gpio::PinDriver<'static, esp_idf_svc::hal::gpio::Input>,
    want: u16,
    timeout_ms: u32,
) -> bool
where
    S: embedded_hal::spi::SpiDevice,
    B: embedded_hal::digital::InputPin<Error = PE>,
    R: embedded_hal::digital::OutputPin<Error = PE>,
    D: embedded_hal::delay::DelayNs,
{
    for _ in 0..timeout_ms {
        if dio1.is_high()
            && let Ok(f) = radio.irq_status()
        {
            let _ = radio.clear_irq(f);
            if f.any(want) {
                return true;
            }
        }
        FreeRtos::delay_ms(1);
    }
    false
}
