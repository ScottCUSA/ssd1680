pub struct Cmd;
#[allow(dead_code)]
impl Cmd {
    pub const DRIVER_OUTPUT_CTRL: u8 = 0x01;
    pub const GATE_VOLTAGE_CTRL: u8 = 0x03;
    pub const SOURCE_VOLTAGE_CTRL: u8 = 0x04;
    pub const DEEP_SLEEP: u8 = 0x10;
    pub const DATA_ENTRY_MODE: u8 = 0x11;
    pub const SW_RESET: u8 = 0x12;
    pub const TEMP_CONTROL: u8 = 0x18;
    pub const MASTER_ACTIVATE: u8 = 0x20;
    pub const DISPLAY_UPDATE_CTRL1: u8 = 0x21;
    pub const DISPLAY_UPDATE_CTRL2: u8 = 0x22;
    pub const WRITE_BW_DATA: u8 = 0x24;
    pub const WRITE_RED_DATA: u8 = 0x26;
    pub const WRITE_VCOM_REG: u8 = 0x2C;
    pub const WRITE_LUT_REG: u8 = 0x32;
    pub const BORDER_WAVEFORM_CTRL: u8 = 0x3C;
    pub const END_OPTION: u8 = 0x3F;
    pub const SET_RAMXPOS: u8 = 0x44;
    pub const SET_RAMYPOS: u8 = 0x45;
    pub const SET_RAMX_COUNTER: u8 = 0x4E;
    pub const SET_RAMY_COUNTER: u8 = 0x4F;
}
