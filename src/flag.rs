pub struct Flag;
#[allow(dead_code)]
impl Flag {
    pub const DATA_ENTRY_INCRY_INCRX: u8 = 0b11;
    pub const INTERNAL_TEMP_SENSOR: u8 = 0x80;
    pub const BORDER_WAVEFORM_FOLLOW_LUT: u8 = 0b0100;
    pub const BORDER_WAVEFORM_LUT0: u8 = 0b0000;
    pub const BORDER_WAVEFORM_LUT1: u8 = 0b0001;
    pub const DISPLAY_MODE_1: u8 = 0xC7;
    pub const DISPLAY_MODE_LOAD_TEMP_1: u8 = 0xF7;
    pub const END_OPTION_NORMAL: u8 = 0x22;
    pub const END_OPTION_KEEP: u8 = 0x07;
}
