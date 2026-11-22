use super::*;
use core::cell::RefCell;
use core::convert::Infallible;
use embedded_hal::digital::ErrorType;
use std::rc::Rc;

/// Behavioural ADS1232 model.
#[expect(
    clippy::struct_excessive_bools,
    reason = "mirrors individual pin levels"
)]
#[derive(Debug, Default)]
struct Chip {
    value: u32, // 24-bit two's complement conversion to shift out
    ready: bool,
    sclk: bool,
    pulses: u32,
    pdwn: bool,
    gain: (bool, bool),
    calibrating: bool,
}

impl Chip {
    fn dout(&self) -> bool {
        if !self.ready || self.pulses == 0 {
            return !self.ready; // DRDY: low when ready
        }
        if self.pulses <= 24 {
            (self.value >> (24 - self.pulses)) & 1 == 1
        } else {
            true // 25th pulse forces DOUT high
        }
    }

    fn rising_edge(&mut self) {
        self.pulses += 1;
        if self.pulses == 26 {
            self.calibrating = true;
        }
    }
}

type Shared = Rc<RefCell<Chip>>;

#[derive(Clone)]
enum Pin {
    Dout(Shared),
    Sclk(Shared),
    Pdwn(Shared),
    Gain0(Shared),
    Gain1(Shared),
}

impl ErrorType for Pin {
    type Error = Infallible;
}

impl InputPin for Pin {
    fn is_high(&mut self) -> Result<bool, Infallible> {
        match self {
            Pin::Dout(c) => Ok(c.borrow().dout()),
            _ => unreachable!(),
        }
    }
    fn is_low(&mut self) -> Result<bool, Infallible> {
        self.is_high().map(|h| !h)
    }
}

impl OutputPin for Pin {
    fn set_high(&mut self) -> Result<(), Infallible> {
        match self {
            Pin::Sclk(c) => {
                let mut c = c.borrow_mut();
                if !c.sclk {
                    c.sclk = true;
                    c.rising_edge();
                }
            }
            Pin::Pdwn(c) => c.borrow_mut().pdwn = true,
            Pin::Gain0(c) => c.borrow_mut().gain.0 = true,
            Pin::Gain1(c) => c.borrow_mut().gain.1 = true,
            Pin::Dout(_) => unreachable!(),
        }
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Infallible> {
        match self {
            Pin::Sclk(c) => c.borrow_mut().sclk = false,
            Pin::Pdwn(c) => c.borrow_mut().pdwn = false,
            Pin::Gain0(c) => c.borrow_mut().gain.0 = false,
            Pin::Gain1(c) => c.borrow_mut().gain.1 = false,
            Pin::Dout(_) => unreachable!(),
        }
        Ok(())
    }
}

struct NoDelay;
impl DelayNs for NoDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

fn setup(gain: Gain) -> (Ads1232<Pin, Pin, Pin, Pin, Pin, NoDelay>, Shared) {
    let c: Shared = Rc::default();
    let pins = Pins {
        dout: Pin::Dout(c.clone()),
        sclk: Pin::Sclk(c.clone()),
        pdwn: Pin::Pdwn(c.clone()),
        gain0: Pin::Gain0(c.clone()),
        gain1: Pin::Gain1(c.clone()),
    };
    (Ads1232::new(pins, NoDelay, gain).unwrap(), c)
}

fn conversion(c: &Shared, value: i32) {
    let mut chip = c.borrow_mut();
    chip.value = (value as u32) & 0x00FF_FFFF;
    chip.ready = true;
    chip.pulses = 0;
}

#[test]
fn gain_pin_table() {
    assert_eq!(Gain::X1.pins(), (false, false));
    assert_eq!(Gain::X2.pins(), (true, false));
    assert_eq!(Gain::X64.pins(), (false, true));
    assert_eq!(Gain::X128.pins(), (true, true));
}

#[test]
fn new_wakes_and_sets_gain() {
    let (_adc, c) = setup(Gain::X128);
    let chip = c.borrow();
    assert!(chip.pdwn);
    assert!(!chip.sclk);
    assert_eq!(chip.gain, (true, true));
}

#[test]
fn set_gain_changes_pins() {
    let (mut adc, c) = setup(Gain::X128);
    adc.set_gain(Gain::X2).unwrap();
    assert_eq!(c.borrow().gain, (true, false));
}

#[test]
fn ready_follows_dout() {
    let (mut adc, c) = setup(Gain::X128);
    assert!(!adc.is_ready().unwrap());
    conversion(&c, 0);
    assert!(adc.is_ready().unwrap());
}

#[test]
fn read_raw_requires_ready() {
    let (mut adc, c) = setup(Gain::X128);
    assert_eq!(adc.read_raw(), Err(Error::NotReady));
    assert_eq!(c.borrow().pulses, 0);
}

#[test]
fn read_raw_positive_negative_and_extremes() {
    let (mut adc, c) = setup(Gain::X128);
    for v in [0, 1, -1, 123_456, -123_456, 0x7F_FFFF, -0x80_0000] {
        conversion(&c, v);
        assert_eq!(adc.read_raw(), Ok(v));
        let chip = c.borrow();
        assert_eq!(chip.pulses, 25, "24 data bits + 1 frame-end pulse");
        assert!(!chip.sclk, "SCLK must idle low");
    }
}

#[test]
fn calibration_sends_26_pulses() {
    let (mut adc, c) = setup(Gain::X128);
    assert_eq!(adc.start_calibration(), Err(Error::NotReady));
    conversion(&c, 42);
    adc.start_calibration().unwrap();
    let chip = c.borrow();
    assert_eq!(chip.pulses, 26);
    assert!(chip.calibrating);
    assert!(!chip.sclk);
}

#[test]
fn power_down_pulls_pdwn_low() {
    let (mut adc, c) = setup(Gain::X128);
    adc.power_down().unwrap();
    assert!(!c.borrow().pdwn);
}
