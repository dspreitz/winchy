//! Barometer at 1 Hz: QNH (seeded from the stored field elevation, then
//! calibrated against GNSS while idle) and the vertical Kalman filter.

use std::sync::Mutex;

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::i2c::I2cDriver;
use winchy_bmp280::Bmp280;
use winchy_fusion::altitude::pressure_to_altitude_m;
use winchy_fusion::vertical::VerticalKalman;
use winchy_rope_logic::qnh::{GnssAltitude, Qnh, QnhConfig, QnhUpdate};
use winchy_settings::NvsStore;

use crate::shared::{State, lock, now_ms};

/// Sample period, ms.
const PERIOD_MS: u32 = 1_000;
/// NVS key of the last saved field elevation (written by the GNSS task).
pub const NVS_LAST_ALT: &str = "last_alt";

fn stored_altitude(store: &Mutex<NvsStore>) -> Option<f32> {
    let mut buf = [0u8; 32];
    lock(store)
        .get_internal(NVS_LAST_ALT, &mut buf)
        .ok()
        .flatten()?
        .parse()
        .ok()
}

/// Task body.
pub fn run(i2c: I2cDriver<'static>, state: &State, store: &Mutex<NvsStore>) {
    let mut bmp = match Bmp280::new(
        i2c,
        winchy_bmp280::ADDRESS,
        winchy_bmp280::Config::default(),
    ) {
        Ok(b) => b,
        Err(e) => {
            log::error!("baro: BMP280 {e:?}");
            return;
        }
    };
    let mut qnh = Qnh::new(QnhConfig::default());
    let mut seeded = false;
    let mut vertical = VerticalKalman::default();
    let mut last_ms = now_ms();
    loop {
        FreeRtos::delay_ms(PERIOD_MS);
        if bmp.start_forced().is_err() {
            continue;
        }
        FreeRtos::delay_ms(10);
        while bmp.is_measuring().unwrap_or(false) {
            FreeRtos::delay_ms(5);
        }
        let m = match bmp.read() {
            Ok(m) if m.is_plausible() => m,
            Ok(_) | Err(_) => continue,
        };
        let p = m.pressure_hpa();
        if !seeded {
            seeded = true;
            if let Some(alt) = stored_altitude(store)
                && qnh.seed(p, alt)
            {
                log::info!(
                    "baro: QNH {:.1} hPa seeded from field elevation {alt:.0} m",
                    qnh.hpa()
                );
            }
        }
        let now = now_ms();
        let (fix, idle, baro_alt) = {
            let s = lock(state);
            let fix = (s.gnss.fix).then(|| GnssAltitude {
                alt_m: s.gnss.alt_m,
                sats: s.gnss.sats,
                hacc_m: s.gnss.hacc_m,
                age_ms: s
                    .gnss
                    .updated_ms
                    .map_or(u64::MAX, |t| now.saturating_sub(t)),
            });
            (fix, s.is_idle(), s.baro.altitude_m)
        };
        if qnh.update(p, fix, baro_alt, idle) == QnhUpdate::Snapped {
            vertical = VerticalKalman::default(); // the jump is not a climb
            log::info!("baro: QNH {:.1} hPa from GNSS", qnh.hpa());
        }
        let dt = (now.saturating_sub(last_ms)) as f32 / 1_000.0;
        last_ms = now;
        vertical.predict(dt);
        vertical.update(pressure_to_altitude_m(p, qnh.hpa()));
        let mut s = lock(state);
        s.baro.pressure_hpa = p;
        s.baro.temperature_c = f32::from(i16::try_from(m.temperature_cdeg).unwrap_or(0)) / 100.0;
        s.baro.altitude_m = vertical.altitude_m();
        s.baro.climb_ms = vertical.climb_rate_ms();
        s.baro.qnh_hpa = qnh.hpa();
        s.baro.qnh_from_gnss = qnh.from_gnss();
        s.baro.updated_ms = Some(now);
    }
}
