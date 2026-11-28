use super::*;

#[test]
fn consecutive_frames_lose_nothing() {
    let mut s = LinkStats::new();
    for seq in 10..15 {
        s.on_frame(seq, -60, 20);
    }
    assert_eq!((s.received(), s.lost()), (5, 0));
    assert_eq!(
        s.report(),
        LinkReport {
            rssi_dbm: -60,
            snr_qdb: 20,
            loss_pct: 0
        }
    );
}

#[test]
fn forward_gap_counts_lost_frames_across_wrap() {
    let mut s = LinkStats::new();
    s.on_frame(0xFFFE, -70, 8);
    s.on_frame(2, -71, 9); // 0xFFFF, 0, 1 missing
    assert_eq!((s.received(), s.lost()), (2, 3));
}

#[test]
fn backward_jump_is_a_restart_not_loss() {
    let mut s = LinkStats::new();
    s.on_frame(500, -70, 8);
    s.on_frame(3, -70, 8);
    s.on_frame(4, -70, 8);
    assert_eq!(s.lost(), 0);
}

#[test]
fn report_smooths_window_loss_and_resets_window() {
    let mut s = LinkStats::new();
    s.on_frame(0, -80, -4);
    s.on_frame(2, -82, -6); // window: 2 received, 1 lost -> 33.3 %
    let r = s.report();
    assert_eq!(r.loss_pct, 10); // 0.3 * 33.3
    assert_eq!((r.rssi_dbm, r.snr_qdb), (-82, -6));
    s.on_frame(3, -82, -6); // window: 1 received, 0 lost
    assert_eq!(s.report().loss_pct, 7); // 0.7 * 10.0
}

#[test]
fn empty_window_decays_the_estimate() {
    let mut s = LinkStats::new();
    s.on_frame(0, -80, 0);
    s.on_frame(4, -80, 0); // 2 received, 3 lost: 60 %
    assert_eq!(s.report().loss_pct, 18);
    assert_eq!(s.report().loss_pct, 13); // empty window counts as 0 %
}
