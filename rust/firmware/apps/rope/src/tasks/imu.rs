//! IMU at 50 Hz: gyro bias, gravity Kalman, rope angle, angle rate and the
//! glider speed estimate.

use std::time::{Duration, Instant};

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriver};
use winchy_fusion::Vec3;
use winchy_fusion::attitude::rope_angle_above_ground_deg;
use winchy_fusion::gravity::GravityKalman;
use winchy_fusion::gyro_bias::{BiasTracker, TrackerConfig, WindowLimits, window_bias};
use winchy_qmi8658::Qmi8658;
use winchy_rope_logic::motion::{
    ANGLE_RATE_ALPHA, ANGLE_RATE_MAX_DPS, AngleRate, segment_speed_ms,
};

use winchy_datalog::record::Record;

use crate::shared::{State, lock, now_ms, now_us};
use crate::tasks::logger::Logger;

/// Sample period.
const PERIOD: Duration = Duration::from_millis(20);
/// Boot bias: windows tried before falling back to zero.
const BIAS_ATTEMPTS: usize = 4;
/// Boot bias: samples per window.
const BIAS_SAMPLES: usize = 50;

type Imu = Qmi8658<SpiDeviceDriver<'static, SpiDriver<'static>>>;

/// Boot-time gyro bias from a still window; zero if the unit is moving (the
/// live tracker heals it once the unit rests).
fn boot_bias(imu: &mut Imu) -> Vec3 {
    let mut window = [[0.0_f32; 3]; BIAS_SAMPLES];
    for attempt in 1..=BIAS_ATTEMPTS {
        let mut n = 0;
        while n < BIAS_SAMPLES {
            if let Ok(s) = imu.read_raw() {
                window[n] = s.gyro_dps();
                n += 1;
            }
            FreeRtos::delay_ms(5);
        }
        let w = window_bias(&window, WindowLimits::default());
        if w.still {
            log::info!("imu: gyro bias {:.2?} dps", w.mean);
            return w.mean;
        }
        log::warn!("imu: bias window {attempt} rejected (moving)");
    }
    log::warn!("imu: no still window, starting from zero bias");
    [0.0; 3]
}

/// Task body.
pub fn run(spi: SpiDeviceDriver<'static, SpiDriver<'static>>, state: &State, logger: &Logger) {
    let mut imu = match Qmi8658::new(spi, &mut FreeRtos) {
        Ok(i) => i,
        Err(e) => {
            log::error!("imu: QMI8658 {e:?}");
            return;
        }
    };
    FreeRtos::delay_ms(50);
    let mut tracker = BiasTracker::new(boot_bias(&mut imu), TrackerConfig::default());
    let mut gravity = GravityKalman::default();
    let mut rate = AngleRate::new(ANGLE_RATE_ALPHA, ANGLE_RATE_MAX_DPS);
    let mut last_us = now_us();
    let mut next = Instant::now();
    let mut errors = 0u32;
    loop {
        next += PERIOD;
        match imu.read_raw() {
            Ok(sample) => {
                let t = now_us();
                logger.log(
                    t,
                    Record::Imu {
                        accel: sample.accel,
                        gyro: sample.gyro,
                    },
                );
                let dt = ((t - last_us) as f32 / 1e6).clamp(0.001, 0.2);
                last_us = t;
                let (accel, raw_gyro) = (sample.accel_g(), sample.gyro_dps());
                let (idle, ground, climb) = {
                    let s = lock(state);
                    (s.is_idle(), s.gnss.ground_speed_ms, s.baro.climb_ms)
                };
                if idle {
                    tracker.update(raw_gyro); // learns only while still
                }
                let b = tracker.bias();
                let gyro = [raw_gyro[0] - b[0], raw_gyro[1] - b[1], raw_gyro[2] - b[2]];
                gravity.predict(gyro, dt);
                gravity.update(accel);
                let angle = rope_angle_above_ground_deg(gravity.gravity());
                let angle_rate = rate.update(angle, dt);
                let mut s = lock(state);
                s.attitude.accel_g = accel;
                s.attitude.gyro_dps = gyro;
                s.attitude.angle_deg = angle;
                s.attitude.angle_rate_dps = angle_rate;
                s.attitude.updated_ms = Some(now_ms());
                s.glider_speed_ms = segment_speed_ms(ground, climb);
            }
            Err(e) => {
                errors += 1;
                if errors % 100 == 1 {
                    log::warn!("imu: read error {e:?} ({errors} total)");
                }
            }
        }
        // Deadline cadence; if we fell behind, resynchronise instead of bursting.
        let now = Instant::now();
        if next > now {
            std::thread::sleep(next - now);
        } else {
            next = now;
        }
    }
}
