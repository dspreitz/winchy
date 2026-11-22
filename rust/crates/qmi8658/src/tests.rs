use super::*;
use embedded_hal_mock::eh1::delay::NoopDelay;
use embedded_hal_mock::eh1::spi::{Mock, Transaction as T};

fn read(reg: u8, data: &[u8]) -> [T<u8>; 4] {
    [
        T::transaction_start(),
        T::write_vec(vec![reg | 0x80]),
        T::read_vec(data.to_vec()),
        T::transaction_end(),
    ]
}

fn write(reg: u8, val: u8) -> [T<u8>; 3] {
    [
        T::transaction_start(),
        T::write_vec(vec![reg & 0x7F, val]),
        T::transaction_end(),
    ]
}

fn init(who_reads: &[u8]) -> Vec<T<u8>> {
    let mut t: Vec<T<u8>> = who_reads.iter().flat_map(|&w| read(0x00, &[w])).collect();
    if who_reads.last() == Some(&WHO_AM_I) {
        for (reg, val) in INIT_SEQUENCE {
            t.extend(write(reg, val));
        }
    }
    t
}

#[test]
fn init_sequence_matches_proven_driver() {
    assert_eq!(
        INIT_SEQUENCE,
        [
            (0x02, 0x60),
            (0x03, 0x05),
            (0x04, 0x45),
            (0x08, 0x83),
            (0x06, 0x55)
        ]
    );
}

#[test]
fn probe_retries_until_id_appears() {
    let imu = Qmi8658::new(Mock::new(&init(&[0x00, 0x00, 0x05])), &mut NoopDelay::new()).unwrap();
    imu.release().done();
}

#[test]
fn probe_gives_up_after_attempts() {
    let reads = vec![0x00; PROBE_ATTEMPTS as usize];
    let mut spi = Mock::new(&init(&reads));
    let r = Qmi8658::new(spi.clone(), &mut NoopDelay::new());
    assert!(matches!(r, Err(Error::NotFound(0x00))));
    spi.done();
}

#[test]
fn burst_decodes_little_endian_signed() {
    let s = RawSample::from_burst(&[
        0x00, 0x40, // ax = 16384
        0x00, 0xC0, // ay = -16384
        0x01, 0x00, // az = 1
        0x80, 0x00, // gx = 128
        0xFF, 0xFF, // gy = -1
        0x00, 0x80, // gz = -32768
    ]);
    assert_eq!(s.accel, [16_384, -16_384, 1]);
    assert_eq!(s.gyro, [128, -1, -32_768]);
}

#[test]
#[allow(clippy::float_cmp, reason = "power-of-two scale factors are exact")]
fn scaling() {
    let s = RawSample {
        accel: [16_384, -8_192, 0],
        gyro: [128, -256, 64],
    };
    assert_eq!(s.accel_g(), [1.0, -0.5, 0.0]);
    assert_eq!(s.gyro_dps(), [1.0, -2.0, 0.5]);
}

#[test]
fn read_raw_bursts_from_ax_l() {
    let burst = [0x00, 0x40, 0, 0, 0, 0, 0x80, 0x00, 0, 0, 0, 0];
    let mut t = init(&[0x05]);
    t.extend(read(0x35, &burst));
    let mut imu = Qmi8658::new(Mock::new(&t), &mut NoopDelay::new()).unwrap();
    assert_eq!(
        imu.read_raw().unwrap(),
        RawSample {
            accel: [16_384, 0, 0],
            gyro: [128, 0, 0]
        }
    );
    imu.release().done();
}

#[test]
fn temperature() {
    let mut t = init(&[0x05]);
    t.extend(read(0x33, &[0x80, 0x19])); // 0x1980 / 256 = 25.5
    let mut imu = Qmi8658::new(Mock::new(&t), &mut NoopDelay::new()).unwrap();
    assert!((imu.temperature_c().unwrap() - 25.5).abs() < 1e-6);
    imu.release().done();
}
