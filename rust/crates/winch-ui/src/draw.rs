//! Layout and drawing.

use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_10X20};
use embedded_graphics::mono_font::{MonoFont, MonoTextStyle};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use winchy_winch_logic::screen::{Header, Screen};

/// Display width, px.
pub const WIDTH: u32 = 128;
/// Display height, px.
pub const HEIGHT: u32 = 64;

/// Clears the display and draws `screen`.
///
/// # Errors
/// Errors of the draw target.
pub fn draw<D: DrawTarget<Color = BinaryColor>>(
    screen: &Screen,
    d: &mut D,
) -> Result<(), D::Error> {
    d.clear(BinaryColor::Off)?;
    header(&screen.header, d)?;
    text(&screen.big[0], &FONT_10X20, Y_BIG0, d)?;
    text(&screen.big[1], &FONT_10X20, Y_BIG1, d)?;
    text(&screen.small, &FONT_6X10, Y_SMALL, d)?;
    text(&screen.bottom, &FONT_6X10, Y_BOTTOM, d)
}

/// Title bar height, px.
const HEADER_H: u32 = 13;
/// Top rows of the body lines, px.
const Y_BIG0: i32 = 14;
const Y_BIG1: i32 = 33;
const Y_SMALL: i32 = 40;
const Y_BOTTOM: i32 = 54;
/// Header text row and small-font advance, px.
const Y_HEADER_TEXT: i32 = 2;
const SMALL_ADVANCE: i32 = 6;

/// 8x8 satellite icon, one byte per row, MSB = leftmost pixel.
const ICON_GPS: [u8; 8] = [0x3C, 0x42, 0x99, 0xA5, 0xA5, 0x99, 0x42, 0x3C];

fn text<D: DrawTarget<Color = BinaryColor>>(
    s: &str,
    font: &MonoFont<'_>,
    y: i32,
    d: &mut D,
) -> Result<(), D::Error> {
    if s.is_empty() {
        return Ok(());
    }
    Text::with_baseline(
        s,
        Point::new(0, y),
        MonoTextStyle::new(font, BinaryColor::On),
        Baseline::Top,
    )
    .draw(d)?;
    Ok(())
}

/// Inverted bar; right to left: "I" (log server reachable), satellite count,
/// GPS icon (struck through without a fix: the geometry is blind).
fn header<D: DrawTarget<Color = BinaryColor>>(h: &Header, d: &mut D) -> Result<(), D::Error> {
    Rectangle::new(Point::zero(), Size::new(WIDTH, HEADER_H))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
        .draw(d)?;
    header_text(2, &h.title, d)?;
    let mut x = i32::try_from(WIDTH).unwrap_or(128);
    if h.online {
        x -= SMALL_ADVANCE;
        header_text(x, "I", d)?;
    }
    let mut digits = [0u8; 3];
    let sats = sats_text(h.sats, &mut digits);
    x -= SMALL_ADVANCE * i32::try_from(sats.len()).unwrap_or(3);
    header_text(x, sats, d)?;
    x -= 10;
    let icon = ICON_GPS.iter().zip(0i32..).flat_map(|(bits, row)| {
        (0..8)
            .filter(move |bit| bits & (0x80 >> bit) != 0)
            .map(move |bit| Pixel(Point::new(x + bit, Y_HEADER_TEXT + row), BinaryColor::Off))
    });
    d.draw_iter(icon)?;
    if !h.fix {
        Line::new(
            Point::new(x, Y_HEADER_TEXT),
            Point::new(x + 7, Y_HEADER_TEXT + 7),
        )
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::Off, 1))
        .draw(d)?;
    }
    Ok(())
}

/// Header text, cut out of the lit bar.
fn header_text<D: DrawTarget<Color = BinaryColor>>(
    x: i32,
    s: &str,
    d: &mut D,
) -> Result<(), D::Error> {
    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::Off);
    Text::with_baseline(s, Point::new(x, Y_HEADER_TEXT), style, Baseline::Top).draw(d)?;
    Ok(())
}

/// Decimal digits of `n` without allocation.
fn sats_text(n: u8, buf: &mut [u8; 3]) -> &str {
    let mut len = 0;
    let mut v = n;
    loop {
        buf[2 - len] = b'0' + v % 10;
        len += 1;
        v /= 10;
        if v == 0 {
            break;
        }
    }
    core::str::from_utf8(&buf[3 - len..]).unwrap_or("?")
}

#[cfg(test)]
mod tests;
