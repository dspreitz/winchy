use super::*;

#[test]
fn decode_any_accepts_both_versions() {
    // v6 LINK_REPORT: version 6, type 5, seq 0, rssi -90 dBm, snr 0, loss 0.
    let r = (-90i16).to_le_bytes();
    assert!(decode_any(&[6, 5, 0, 0, r[0], r[1], 0, 0]).is_some());
    let mut buf = [0u8; 32];
    let n = Frame {
        seq: 1,
        message: Message::UploadAck { nonce: 4 },
    }
    .encode(&mut buf)
    .unwrap();
    assert_eq!(
        decode_any(&buf[..n]).unwrap().message,
        Message::UploadAck { nonce: 4 }
    );
    assert_eq!(decode_any(&[9, 9, 9, 9]), None);
}
