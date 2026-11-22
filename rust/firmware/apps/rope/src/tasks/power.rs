//! Power supervision every 5 s (battery, charging, low-battery warning) and a
//! compact status line for the log.

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::i2c::I2cDriver;
use winchy_axp2101::Axp2101;
use winchy_rope_logic::battery::BatteryMonitor;

use winchy_datalog::record::Record;

use crate::shared::{State, lock, now_ms};
use crate::tasks::logger::Logger;

/// Period, ms.
const PERIOD_MS: u32 = 5_000;
/// Status line every Nth period.
const STATUS_EVERY: u32 = 2;

fn log_status(state: &State) {
    let s = *lock(state);
    let age = |t: Option<u64>| t.map_or(u64::MAX, |t| now_ms().saturating_sub(t));
    log::info!(
        "status: force {} (err {}) | angle {:.1} deg | baro {:.1} hPa alt {:.1} m climb {:+.1} m/s qnh {:.1}{} | gnss {} {} sats hAcc {:.1} m | v {:.1} m/s | clock {:?} | batt {:?} mV {:?} % chg {} low {} | radio {} dBm tx {} rx {} report {:?} | winch {}",
        s.force.tared(),
        s.force.errors,
        s.attitude.angle_deg,
        s.baro.pressure_hpa,
        s.baro.altitude_m.unwrap_or(0.0),
        s.baro.climb_ms,
        s.baro.qnh_hpa,
        if s.baro.qnh_from_gnss { " (gnss)" } else { "" },
        if s.gnss.fix { "fix" } else { "nofix" },
        s.gnss.sats,
        s.gnss.hacc_m,
        s.glider_speed_ms,
        s.clock,
        s.power.battery_mv,
        s.power.battery_pct,
        s.power.charging,
        s.power.battery_low,
        s.link.tx_power_dbm,
        s.link.tx_count,
        s.link.rx_count,
        s.link
            .last_report
            .filter(|_| age(s.link.report_ms) < 10_000),
        s.geometry
            .map_or_else(|| "-".to_owned(), |g| format!("{:.0} m", g.slant_m)),
    );
}

/// Task body.
pub fn run(mut pmu: Axp2101<I2cDriver<'static>>, state: &State, logger: &Logger) {
    let mut monitor = BatteryMonitor::default();
    let mut n = 0u32;
    loop {
        let battery_mv = pmu.battery_mv().ok().flatten();
        let battery_pct = pmu.battery_percent().ok().flatten();
        let system_mv = pmu.system_mv().unwrap_or(0);
        let charging = pmu.is_charging().unwrap_or(false);
        logger.log_now(Record::Power {
            battery_mv: battery_mv.unwrap_or(0),
            battery_pct: battery_pct.unwrap_or(u8::MAX),
            system_mv,
            charging,
        });
        {
            let mut s = lock(state);
            let low = monitor.update(battery_mv, s.is_idle());
            if low && !s.power.battery_low {
                log::warn!("power: battery low ({battery_mv:?} mV)");
            }
            s.power.system_mv = system_mv;
            s.power.battery_mv = battery_mv;
            s.power.battery_pct = battery_pct;
            s.power.charging = charging;
            s.power.battery_low = low;
        }
        n += 1;
        if n % STATUS_EVERY == 1 {
            log_status(state);
        }
        FreeRtos::delay_ms(PERIOD_MS);
    }
}
