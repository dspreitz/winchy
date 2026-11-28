use super::*;
use core::convert::Infallible;
use heapless::String;
use winchy_winch_logic::screen::Header;

/// 128x64 test framebuffer; counts writes outside the panel.
struct Fb {
    px: [[bool; 128]; 64],
    outside: u32,
}

impl Fb {
    fn new() -> Self {
        Self {
            px: [[true; 128]; 64], // garbage: draw must clear
            outside: 0,
        }
    }

    fn on(&self, x: usize, y: usize) -> bool {
        self.px[y][x]
    }

    /// Lit pixels in rows `y0..y1`.
    fn lit(&self, y0: usize, y1: usize) -> usize {
        self.px[y0..y1].iter().flatten().filter(|p| **p).count()
    }
}

impl OriginDimensions for Fb {
    fn size(&self) -> Size {
        Size::new(128, 64)
    }
}

impl DrawTarget for Fb {
    type Color = BinaryColor;
    type Error = Infallible;

    fn draw_iter<I: IntoIterator<Item = Pixel<BinaryColor>>>(
        &mut self,
        pixels: I,
    ) -> Result<(), Infallible> {
        for Pixel(p, c) in pixels {
            match (usize::try_from(p.x), usize::try_from(p.y)) {
                (Ok(x), Ok(y)) if x < 128 && y < 64 => self.px[y][x] = c.is_on(),
                _ => self.outside += 1,
            }
        }
        Ok(())
    }
}

fn s<const N: usize>(t: &str) -> String<N> {
    let mut v = String::new();
    v.push_str(t).unwrap();
    v
}

fn screen(big0: &str, big1: &str, small: &str, bottom: &str, fix: bool) -> Screen {
    Screen {
        header: Header {
            title: s("CLIMB"),
            sats: 9,
            fix,
            online: false,
        },
        big: [s(big0), s(big1)],
        small: s(small),
        bottom: s(bottom),
    }
}

fn render(sc: &Screen) -> Fb {
    let mut fb = Fb::new();
    draw(sc, &mut fb).unwrap();
    fb
}

#[test]
fn title_bar_is_inverted_and_body_cleared() {
    let fb = render(&screen("", "", "", "", true));
    assert!(fb.on(0, 0) && fb.on(127, 12)); // bar background lit
    assert!(fb.lit(0, 13) < 128 * 13); // title text cut out of it
    assert_eq!(fb.lit(13, 64), 0); // nothing else drawn
    assert_eq!(fb.outside, 0);
}

#[test]
fn every_line_lands_in_its_band() {
    let fb = render(&screen("90km/h", "", "", "", true));
    assert!(fb.lit(14, 34) > 0 && fb.lit(34, 64) == 0);
    let fb = render(&screen("", "1235N", "", "", true));
    assert!(fb.lit(33, 53) > 0 && fb.lit(13, 33) == 0 && fb.lit(54, 64) == 0);
    let fb = render(&screen("", "", "n=12", "", true));
    assert!(fb.lit(40, 50) > 0 && fb.lit(13, 40) == 0 && fb.lit(54, 64) == 0);
    let fb = render(&screen("", "", "", "A44 H1200 -97dB", true));
    assert!(fb.lit(54, 64) > 0 && fb.lit(13, 54) == 0);
}

#[test]
fn widest_lines_fit_the_panel() {
    let fb = render(&screen(
        "888888888888",
        "-108k",
        "n=99999",
        "!BATT LOW 3.4V A44H",
        true,
    ));
    assert_eq!(fb.outside, 0);
}

#[test]
fn missing_fix_strikes_through_the_gps_icon() {
    let with = render(&screen("", "", "", "", true));
    let without = render(&screen("", "", "", "", false));
    assert_ne!(with.px[..13], without.px[..13]);
}
