//! OLED at 2 Hz: telemetry while the rope talks, otherwise the winch's own
//! GPS/survey view (layout in `winchy-winch-ui`, content in `winch-logic`).

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::i2c::I2cDriver;
use ssd1306::prelude::*;
use ssd1306::{I2CDisplayInterface, Ssd1306};
use winchy_winch_logic::screen;

use crate::shared::{State, lock, now_ms};

/// Frame period, ms.
const PERIOD_MS: u32 = 500;

/// Task body.
pub fn run(i2c: I2cDriver<'static>, state: &State) {
    let mut display = Ssd1306::new(
        I2CDisplayInterface::new(i2c),
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    if let Err(e) = display.init() {
        log::error!("display: SSD1306 init {e:?}");
        return;
    }
    log::info!("display: SSD1306 up");
    let mut frame = 0u32;
    let mut errors = 0u32;
    loop {
        let sc = screen::build(&lock(state), now_ms(), frame);
        frame = frame.wrapping_add(1);
        let drawn = winchy_winch_ui::draw(&sc, &mut display);
        if drawn.is_err() || display.flush().is_err() {
            errors += 1;
            if errors % 20 == 1 {
                log::warn!("display: I2C error ({errors} total)");
            }
        }
        FreeRtos::delay_ms(PERIOD_MS);
    }
}
