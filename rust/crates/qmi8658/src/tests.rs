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

/// `CTRL9` command with handshake: write, poll done (after `busy` polls),
/// acknowledge, poll cleared.
fn command(cmd: u8, busy: usize) -> Vec<T<u8>> {
    let mut t = write(0x0A, cmd).to_vec();
    for _ in 0..busy {
        t.extend(read(0x2D, &[0x00]));
    }
    t.extend(read(0x2D, &[0x80]));
    t.extend(write(0x0A, 0x00));
    t.extend(read(0x2D, &[0x00]));
    t
}

#[test]
fn enable_fifo_configures_stream_mode_with_sensors_off() {
    let mut t = init(&[0x05]);
    t.extend(write(0x09, 0x80)); // handshake via STATUSINT
    t.extend(write(0x08, 0x00)); // sensors off
    t.extend(write(0x14, 0x0E)); // 128 frames, stream
    t.extend(command(0x04, 2)); // reset FIFO
    t.extend(write(0x08, 0x03)); // sensors on, SyncSample off
    let mut imu = Qmi8658::new(Mock::new(&t), &mut NoopDelay::new()).unwrap();
    imu.enable_fifo().unwrap();
    imu.release().done();
}

#[test]
fn hung_command_times_out() {
    let mut t = init(&[0x05]);
    t.extend(write(0x09, 0x80));
    t.extend(write(0x08, 0x00));
    t.extend(write(0x14, 0x0E));
    t.extend(write(0x0A, 0x04));
    for _ in 0..CMD_POLLS {
        t.extend(read(0x2D, &[0x00]));
    }
    let mut spi = Mock::new(&t);
    let mut imu = Qmi8658::new(spi.clone(), &mut NoopDelay::new()).unwrap();
    assert_eq!(imu.enable_fifo(), Err(Error::Timeout));
    spi.done();
}

/// Frame `i` with recognisable values.
fn frame(i: i16) -> ([u8; 12], RawSample) {
    let s = RawSample {
        accel: [i, -i, 100 + i],
        gyro: [1000 + i, -1000 - i, i * 3],
    };
    let mut b = [0u8; 12];
    for (k, v) in s.accel.iter().chain(s.gyro.iter()).enumerate() {
        b[2 * k..2 * k + 2].copy_from_slice(&v.to_le_bytes());
    }
    (b, s)
}

#[test]
fn read_fifo_reads_complete_frames_in_one_burst() {
    let mut t = init(&[0x05]);
    t.extend(command(0x05, 0));
    // 6 frames + 1 partial word = 37 words.
    t.extend(read(0x15, &[37, 0x10]));
    let frames: Vec<_> = (0..6).map(frame).collect();
    let bytes: Vec<u8> = frames.iter().flat_map(|f| f.0).collect();
    t.extend(read(0x17, &bytes)); // one burst, CS held
    t.extend(write(0x14, 0x0E)); // leave read mode
    let mut imu = Qmi8658::new(Mock::new(&t), &mut NoopDelay::new()).unwrap();
    let mut out = [RawSample::default(); FIFO_FRAMES];
    let batch = imu.read_fifo(&mut out, || 42).unwrap();
    assert_eq!(
        batch,
        FifoBatch {
            frames: 6,
            overflow: false,
            stamp: 42
        }
    );
    for (got, want) in out.iter().zip(&frames) {
        assert_eq!(*got, want.1);
    }
    imu.release().done();
}

#[test]
fn read_fifo_uses_count_msbs_and_flags_loss() {
    let mut t = init(&[0x05]);
    t.extend(command(0x05, 1));
    // Full FIFO: 768 words = 0x300, status FULL.
    t.extend(read(0x15, &[0x00, 0x83]));
    let bytes = vec![0u8; FIFO_FRAMES * 12];
    t.extend(read(0x17, &bytes));
    t.extend(write(0x14, 0x0E));
    let mut imu = Qmi8658::new(Mock::new(&t), &mut NoopDelay::new()).unwrap();
    let mut out = [RawSample::default(); FIFO_FRAMES];
    let batch = imu.read_fifo(&mut out, || ()).unwrap();
    assert_eq!(batch.frames, FIFO_FRAMES);
    assert!(batch.overflow);
    imu.release().done();
}

#[test]
fn empty_fifo_reads_no_data() {
    let mut t = init(&[0x05]);
    t.extend(command(0x05, 0));
    t.extend(read(0x15, &[0, 0x00]));
    t.extend(write(0x14, 0x0E));
    let mut imu = Qmi8658::new(Mock::new(&t), &mut NoopDelay::new()).unwrap();
    let mut out = [RawSample::default(); FIFO_FRAMES];
    assert_eq!(imu.read_fifo(&mut out, || ()).unwrap().frames, 0);
    imu.release().done();
}
