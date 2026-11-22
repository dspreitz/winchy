use super::*;
use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};

/// Datasheet example calibration (`dig_T1..dig_P9`), little-endian.
const CAL: [u8; 24] = [
    0x70, 0x6b, 0x43, 0x67, 0x18, 0xfc, 0x7d, 0x8e, 0x43, 0xd6, 0xd0, 0x0b, 0x27, 0x0b, 0x8c, 0x00,
    0xf9, 0xff, 0x8c, 0x3c, 0xf8, 0xc6, 0x70, 0x17,
];
const A: u8 = ADDRESS;

fn init_transactions(id: u8) -> Vec<T> {
    vec![
        T::write_read(A, vec![0xD0], vec![id]),
        T::write_read(A, vec![0x88], CAL.to_vec()),
        T::write(A, vec![0xF5, (3 << 5) | (1 << 2)]),
        T::write(A, vec![0xF4, (4 << 5) | (5 << 2)]),
    ]
}

#[test]
fn parses_datasheet_calibration() {
    let c = Calibration::from_bytes(&CAL);
    assert_eq!(c.t, (27504, 26435, -1000));
    assert_eq!(
        c.p,
        (36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000)
    );
}

#[test]
fn datasheet_compensation_is_bit_exact() {
    let m = Calibration::from_bytes(&CAL).compensate(519_888, 415_148);
    assert_eq!(m.temperature_cdeg, 2508);
    assert_eq!(m.pressure_q24_8, 25_767_233);
    assert!((m.pressure_hpa() - 1_006.532_5).abs() < 1e-3);
}

#[test]
fn degenerate_calibration_does_not_divide_by_zero() {
    let mut c = Calibration::from_bytes(&CAL);
    c.p.0 = 0; // dig_P1 = 0 -> var1 = 0
    assert_eq!(c.compensate(519_888, 415_148).pressure_q24_8, 0);
}

#[test]
fn plausibility_window() {
    let m = |hpa: f64| Measurement {
        temperature_cdeg: 0,
        pressure_q24_8: (hpa * 25_600.0) as u32,
    };
    assert!(m(1013.25).is_plausible());
    assert!(!m(0.0).is_plausible());
    assert!(!m(299.0).is_plausible());
    assert!(!m(1101.0).is_plausible());
}

#[test]
fn init_configures_sleep_mode() {
    let mut bmp = Bmp280::new(Mock::new(&init_transactions(0x58)), A, Config::default()).unwrap();
    assert_eq!(bmp.calibration(), Calibration::from_bytes(&CAL));
    bmp.i2c.done();
}

#[test]
fn accepts_bme280_rejects_others() {
    let bmp = Bmp280::new(Mock::new(&init_transactions(0x60)), A, Config::default());
    bmp.unwrap().release().done();
    let mut mock = Mock::new(&[T::write_read(A, vec![0xD0], vec![0x55])]);
    assert!(matches!(
        Bmp280::new(mock.clone(), A, Config::default()),
        Err(Error::NotFound(0x55))
    ));
    mock.done();
}

#[test]
fn forced_measurement_cycle() {
    let ctrl = (4 << 5) | (5 << 2);
    let mut t = init_transactions(0x58);
    t.extend([
        T::write(A, vec![0xF4, ctrl | 0x01]),
        T::write_read(A, vec![0xF3], vec![0x08]),
        T::write_read(A, vec![0xF3], vec![0x00]),
        T::write_read(A, vec![0xF7], vec![0x65, 0x5A, 0xC0, 0x7E, 0xED, 0x00]),
    ]);
    let mut bmp = Bmp280::new(Mock::new(&t), A, Config::default()).unwrap();
    bmp.start_forced().unwrap();
    assert!(bmp.is_measuring().unwrap());
    assert!(!bmp.is_measuring().unwrap());
    let m = bmp.read().unwrap();
    assert_eq!((m.temperature_cdeg, m.pressure_q24_8), (2508, 25_767_233));
    bmp.release().done();
}
