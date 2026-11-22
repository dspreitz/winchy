//! Load cell: offset calibration and tare at boot, then free-running at the
//! ADS1232 rate (10 SPS), with periodic offset recalibration while idle.

use esp_idf_svc::hal::delay::{Ets, FreeRtos};
use winchy_ads1232::{Ads1232, Gain};
use winchy_rope_logic::force::{RecalPolicy, TARE_SAMPLES, tare};

use winchy_datalog::record::{Record, event};

use crate::shared::{State, lock, now_ms, now_us};
use crate::tasks::logger::Logger;

/// No conversion within this means a DRDY timeout, ms.
const TIMEOUT_MS: u32 = 1_000;

type Adc = Ads1232<
    esp_idf_svc::hal::gpio::PinDriver<'static, esp_idf_svc::hal::gpio::Input>,
    esp_idf_svc::hal::gpio::PinDriver<'static, esp_idf_svc::hal::gpio::Output>,
    esp_idf_svc::hal::gpio::PinDriver<'static, esp_idf_svc::hal::gpio::Output>,
    esp_idf_svc::hal::gpio::PinDriver<'static, esp_idf_svc::hal::gpio::Output>,
    esp_idf_svc::hal::gpio::PinDriver<'static, esp_idf_svc::hal::gpio::Output>,
    Ets,
>;

fn wait_ready(adc: &mut Adc, timeout_ms: u32) -> bool {
    for _ in 0..timeout_ms / 2 {
        if adc.is_ready().unwrap_or(false) {
            return true;
        }
        FreeRtos::delay_ms(2);
    }
    false
}

/// One conversion and the monotonic time it was read, µs.
fn read(adc: &mut Adc) -> Option<(i32, i64)> {
    if !wait_ready(adc, TIMEOUT_MS) {
        return None;
    }
    // Keep the 25 SCLK pulses unbroken (SCLK high > 100 µs = standby).
    let v = esp_idf_svc::hal::interrupt::free(|| adc.read_raw()).ok()?;
    Some((v, now_us()))
}

fn calibrate(adc: &mut Adc) -> bool {
    if !wait_ready(adc, TIMEOUT_MS) || adc.start_calibration().is_err() {
        return false;
    }
    FreeRtos::delay_ms(5);
    wait_ready(adc, 1_500)
}

/// Task body.
pub fn run(pins: winchy_board::AdsPins, state: &State, logger: &Logger) {
    let mut adc = match Ads1232::new(pins, Ets, Gain::X128) {
        Ok(a) => a,
        Err(e) => {
            log::error!("force: ADS1232 {e:?}");
            return;
        }
    };
    if !calibrate(&mut adc) {
        // A missing load cell must not stop the rest of the unit.
        log::warn!("force: no ADS1232 conversion, continuing without force");
        return;
    }
    let mut samples = [0i32; TARE_SAMPLES];
    let mut n = 0;
    while n < TARE_SAMPLES {
        match read(&mut adc) {
            Some((v, _)) => {
                samples[n] = v;
                n += 1;
            }
            None => break,
        }
    }
    let offset = tare(&samples[..n]).unwrap_or(0);
    log::info!("force: offset-calibrated, tared at {offset} ({n} samples)");
    lock(state).force.offset = offset;
    logger.event(event::TARE, offset);

    let policy = RecalPolicy::default();
    let mut last_recal = now_ms();
    loop {
        let raw = read(&mut adc);
        let now = now_ms();
        let (tared, idle) = {
            let mut s = lock(state);
            match raw {
                Some((v, t)) => {
                    logger.log(t, Record::Force { raw: v });
                    s.force.raw = v;
                    s.force.updated_ms = Some(now);
                }
                None => s.force.errors = s.force.errors.wrapping_add(1),
            }
            (s.force.tared(), s.is_idle())
        };
        if policy.due(now, last_recal, tared, idle) {
            if !calibrate(&mut adc) {
                lock(state).force.errors += 1;
            }
            last_recal = now_ms();
        }
    }
}
