use super::*;
use embedded_hal_mock::eh1::i2c::{Mock, Transaction as T};

const A: u8 = ADDRESS;

fn rmw(reg: u8, old: u8, new: u8) -> [T; 2] {
    [
        T::write_read(A, vec![reg], vec![old]),
        T::write(A, vec![reg, new]),
    ]
}

fn run(expect: &[T], f: impl FnOnce(&mut Axp2101<Mock>)) {
    let mut pmu = Axp2101::new(Mock::new(expect));
    f(&mut pmu);
    pmu.release().done();
}

#[test]
fn probe_accepts_chip_id() {
    run(&[T::write_read(A, vec![0x03], vec![0x4A])], |p| {
        assert_eq!(p.probe(), Ok(()));
    });
}

#[test]
fn probe_rejects_other_id() {
    run(&[T::write_read(A, vec![0x03], vec![0x47])], |p| {
        assert_eq!(p.probe(), Err(Error::NotFound(0x47)));
    });
}

#[test]
fn set_voltage_dc1_writes_whole_register() {
    run(&[T::write(A, vec![0x82, 18])], |p| {
        p.set_voltage(Rail::Dc1, 3300).unwrap();
    });
}

#[test]
fn set_voltage_preserves_reserved_bits() {
    run(&rmw(0x84, 0xFF, 0x80 | 0x69 /* 105 = 3.3 V */), |p| {
        p.set_voltage(Rail::Dc3, 3300).unwrap();
    });
    run(&rmw(0x93, 0xE5, 0xE0 | 0x1C /* 28 = 3.3 V */), |p| {
        p.set_voltage(Rail::Aldo2, 3300).unwrap();
    });
}

#[test]
fn set_voltage_invalid_writes_nothing() {
    run(&[], |p| {
        assert_eq!(
            p.set_voltage(Rail::Dc1, 5000),
            Err(Error::Voltage(VoltageError::OutOfRange))
        );
    });
}

#[test]
fn set_enabled_sets_and_clears_bit() {
    run(&rmw(0x80, 0b0000_0001, 0b0001_0001), |p| {
        p.set_enabled(Rail::Dc5, true).unwrap();
    });
    run(&rmw(0x90, 0xFF, 0x7F), |p| {
        p.set_enabled(Rail::Dldo1, false).unwrap();
    });
    run(&rmw(0x91, 0x01, 0x00), |p| {
        p.set_enabled(Rail::Dldo2, false).unwrap();
    });
}

#[test]
fn vbus_limits() {
    let mut t = rmw(0x15, 0xA0, 0xA6).to_vec();
    t.extend(rmw(0x16, 0xF0, 0xF4));
    run(&t, |p| {
        p.set_vbus_limits(VbusVoltageLimit::Mv4360, VbusCurrentLimit::Ma1500)
            .unwrap();
    });
}

#[test]
fn power_down_voltage() {
    run(&rmw(0x24, 0xFF, 0xF8), |p| {
        p.set_power_down_voltage(2600).unwrap();
    });
    run(&[], |p| {
        assert_eq!(
            p.set_power_down_voltage(2500),
            Err(Error::Voltage(VoltageError::OutOfRange))
        );
    });
}

#[test]
fn power_key_long_press_powers_off() {
    let mut t = rmw(0x27, 0x00, (1 << 2) | 3).to_vec();
    t.extend(rmw(0x22, 0b11, 0b10)); // bit0 clear = power off; bit1 set = enable
    run(&t, |p| {
        p.set_power_key(PowerOffTime::S6, PowerOnTime::S2).unwrap();
    });
}

#[test]
fn adc_channels() {
    let ch = AdcChannels {
        battery: true,
        ts_pin: false,
        vbus: true,
        system: true,
    };
    run(&rmw(0x30, 0b1111_0010, 0b1111_1101), |p| {
        p.set_adc_channels(ch).unwrap();
    });
}

#[test]
fn battery_detection() {
    run(&rmw(0x68, 0x00, 0x01), |p| {
        p.enable_battery_detection().unwrap();
    });
}

#[test]
fn charge_led_manual_on() {
    // keep 0xC8, manual-control bits 0x05, mode in bits 4..5
    run(&rmw(0x69, 0xFF, 0xC8 | 0x05 | (3 << 4)), |p| {
        p.set_charge_led(ChargeLed::On).unwrap();
    });
}

#[test]
fn backup_battery_charge() {
    let mut t = rmw(0x6A, 0xF0, 0xF7).to_vec();
    t.extend(rmw(0x18, 0x00, 0x04));
    run(&t, |p| p.enable_backup_battery_charge(3300).unwrap());
}

#[test]
fn charger() {
    let mut t = rmw(0x61, 0xFF, 0xFE).to_vec();
    t.extend(rmw(0x62, 0xFF, 0xE0 | 0x10 /* 16 = 1000 mA */));
    t.extend(rmw(0x63, 0xFF, 0xF1));
    t.extend(rmw(0x64, 0xFF, 0xFE));
    let c = Charger {
        precharge: 2,
        constant_current: 16,
        termination: 1,
        target_voltage: 2,
    };
    run(&t, |p| p.set_charger(c).unwrap());
}

#[test]
fn irq_mask_writes_each_register_and_clears_status() {
    let mask = irq::BAT_INSERT | irq::BAT_REMOVE | irq::BAT_CHG_DONE;
    run(
        &[
            T::write(A, vec![0x40, 0x00]),
            T::write(A, vec![0x41, 0x30]),
            T::write(A, vec![0x42, 0x10]),
            T::write(A, vec![0x48, 0xFF]),
            T::write(A, vec![0x49, 0xFF]),
            T::write(A, vec![0x4A, 0xFF]),
        ],
        |p| p.set_irq_mask(mask).unwrap(),
    );
}

#[test]
fn watchdog_disable() {
    run(&rmw(0x18, 0x0F, 0x0E), |p| p.disable_watchdog().unwrap());
}

#[test]
fn battery_readings_with_battery() {
    run(
        &[
            T::write_read(A, vec![0x00], vec![0x08]),
            T::write_read(A, vec![0x34], vec![0xEF]),
            T::write_read(A, vec![0x35], vec![0x92]),
        ],
        |p| assert_eq!(p.battery_mv(), Ok(Some(0x0F92))),
    );
    run(
        &[
            T::write_read(A, vec![0x00], vec![0x08]),
            T::write_read(A, vec![0xA4], vec![87]),
        ],
        |p| assert_eq!(p.battery_percent(), Ok(Some(87))),
    );
}

#[test]
fn battery_readings_without_battery() {
    run(&[T::write_read(A, vec![0x00], vec![0x00])], |p| {
        assert_eq!(p.battery_mv(), Ok(None));
    });
    run(&[T::write_read(A, vec![0x00], vec![0x00])], |p| {
        assert_eq!(p.battery_percent(), Ok(None));
    });
}

#[test]
fn system_voltage_masks_high_bits() {
    let t = [
        T::write_read(A, vec![0x3A], vec![0xD4]),
        T::write_read(A, vec![0x3B], vec![0x10]),
    ];
    run(&t, |p| {
        assert_eq!(p.system_mv(), Ok(0x1410));
    });
}

#[test]
fn charging_state() {
    run(&[T::write_read(A, vec![0x01], vec![0b0010_0000])], |p| {
        assert_eq!(p.is_charging(), Ok(true));
    });
    run(&[T::write_read(A, vec![0x01], vec![0b0100_0000])], |p| {
        assert_eq!(p.is_charging(), Ok(false));
    });
}
