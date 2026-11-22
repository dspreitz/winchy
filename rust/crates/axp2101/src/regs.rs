//! Register addresses.

pub const STATUS1: u8 = 0x00;
pub const STATUS2: u8 = 0x01;
pub const IC_TYPE: u8 = 0x03;
pub const CHARGE_GAUGE_WDT_CTRL: u8 = 0x18;
pub const INPUT_VOL_LIMIT_CTRL: u8 = 0x15;
pub const INPUT_CUR_LIMIT_CTRL: u8 = 0x16;
pub const PWROFF_EN: u8 = 0x22;
pub const VOFF_SET: u8 = 0x24;
pub const IRQ_OFF_ON_LEVEL_CTRL: u8 = 0x27;
pub const ADC_CHANNEL_CTRL: u8 = 0x30;
pub const ADC_VBAT_H: u8 = 0x34;
pub const ADC_VSYS_H: u8 = 0x3A;
pub const INTEN1: u8 = 0x40;
pub const INTSTS1: u8 = 0x48;
pub const IPRECHG_SET: u8 = 0x61;
pub const ICC_CHG_SET: u8 = 0x62;
pub const ITERM_CHG_SET_CTRL: u8 = 0x63;
pub const CV_CHG_VOL_SET: u8 = 0x64;
pub const BAT_DET_CTRL: u8 = 0x68;
pub const CHGLED_SET_CTRL: u8 = 0x69;
pub const BTN_BAT_CHG_VOL_SET: u8 = 0x6A;
pub const DC_ONOFF_DVM_CTRL: u8 = 0x80;
pub const LDO_ONOFF_CTRL0: u8 = 0x90;
pub const LDO_ONOFF_CTRL1: u8 = 0x91;
pub const BAT_PERCENT_DATA: u8 = 0xA4;
