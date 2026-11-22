//! Zombie-link probe: one DNS query to the gateway; ANY reply proves two-way
//! traffic (a "connected" STA can sit with IP/RSSI looking healthy while no
//! packet moves - seen on the bench 2026-07-07).

/// Length of [`probe_query`].
pub const QUERY_LEN: usize = 28;

/// A DNS A-record query for `github.com` with transaction `id`.
#[must_use]
pub fn probe_query(id: u16) -> [u8; QUERY_LEN] {
    let [hi, lo] = id.to_be_bytes();
    [
        hi, lo, 0x01, 0x00, // id, flags: standard query, recursion desired
        0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 1 question
        6, b'g', b'i', b't', b'h', b'u', b'b', 3, b'c', b'o', b'm', 0, 0x00, 0x01, 0x00,
        0x01, // type A, class IN
    ]
}

/// True if `reply` is a DNS response to transaction `id`.
#[must_use]
pub fn is_reply(reply: &[u8], id: u16) -> bool {
    reply.len() >= 12 && reply[..2] == id.to_be_bytes() && reply[2] & 0x80 != 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn query_layout() {
        let q = probe_query(0x1234);
        assert_eq!(
            q,
            [
                0x12, 0x34, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, // header
                6, b'g', b'i', b't', b'h', b'u', b'b', 3, b'c', b'o', b'm', 0, // name
                0x00, 0x01, 0x00, 0x01, // A, IN
            ]
        );
    }

    #[test]
    fn reply_matching() {
        let mut r = probe_query(0xBEEF).to_vec();
        r[2] |= 0x80; // QR = response
        assert!(is_reply(&r, 0xBEEF));
        assert!(!is_reply(&r, 0xBEEE), "other transaction");
        assert!(
            !is_reply(&probe_query(0xBEEF), 0xBEEF),
            "a query is not a reply"
        );
        assert!(!is_reply(&r[..11], 0xBEEF), "truncated");
    }
}
