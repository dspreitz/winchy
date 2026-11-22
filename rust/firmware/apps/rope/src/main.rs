//! Winchy rope unit firmware. Currently a hardware bring-up diagnostic:
//! power profile, then one round of readings from every sensor.

use esp_idf_svc::hal::delay::{Ets, FreeRtos};
use esp_idf_svc::hal::peripherals::Peripherals;
use winchy_ads1232::{Ads1232, Gain};
use winchy_bmp280::Bmp280;
use winchy_board::RopeBoard;
use winchy_fusion::attitude::rope_angle_above_ground_deg;
use winchy_qmi8658::Qmi8658;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("winchy-rope {} bring-up", env!("CARGO_PKG_VERSION"));

    let Some(p) = Peripherals::take().ok() else {
        log::error!("peripherals already taken");
        return;
    };
    let mut board = match RopeBoard::init(p.i2c0, p.i2c1, p.spi2) {
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
        ..
    } = board;
    report_baro(sensor_i2c);
    report_imu(imu_spi);
    report_force(ads);
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
