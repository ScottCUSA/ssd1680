use super::{buffer_len, Display};
use super::{DisplayRotation, InitStep};
use crate::cmd::Cmd;
use crate::color::Color;
use crate::{driver::Ssd1680Driver, flag, interface::SpiDisplayInterface};
use display_interface::DisplayError;
use embedded_graphics::{
    pixelcolor::{BinaryColor, Gray2},
    prelude::*,
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;
use log::debug;

/// Display width for 2.9in display
pub const WIDTH: u16 = 128;
/// Display height for 2.9in display
pub const HEIGHT: u16 = 296;

/// Initialization sequence for the 2.9in panel mono full update
pub const INIT_SEQUENCE_MONOFULL: &[InitStep] = &[
    // Set Initialial Configuration
    InitStep::SWReset,
    InitStep::DelayMs(10),
    // Send Initialization Code 0x01, 0x11, 0x44, 0x45, 0x3C
    InitStep::CmdData(Cmd::DRIVER_OUTPUT_CTRL, &[0x27, 0x01, 0x00]),
    InitStep::CmdData(Cmd::DATA_ENTRY_MODE, &[flag::Flag::DATA_ENTRY_INCRY_INCRX]),
    InitStep::CmdData(Cmd::SET_RAMXPOS, &[0x00, 0x0F]),
    InitStep::CmdData(Cmd::SET_RAMYPOS, &[0x00, 0x00, 0x27, 0x01]),
    InitStep::CmdData(
        Cmd::BORDER_WAVEFORM_CTRL,
        &[flag::Flag::BORDER_WAVEFORM_FOLLOW_LUT | flag::Flag::BORDER_WAVEFORM_LUT1],
    ),
    // Load Waveform LUT
    InitStep::CmdData(Cmd::TEMP_CONTROL, &[flag::Flag::INTERNAL_TEMP_SENSOR]),
    InitStep::CmdData(Cmd::DISPLAY_UPDATE_CTRL1, &[0x00, 0x80]),
    // InitStep::CmdData(Cmd::DISPLAY_UPDATE_CTRL2, &[0x20]), // load default LUT (default mono)
    // Used for grayscale LUT (not yet implemented)
    // load LUT into memory instead!
    InitStep::CmdData(Cmd::END_OPTION, &[flag::Flag::END_OPTION_NORMAL]),
    InitStep::CmdData(Cmd::GATE_VOLTAGE_CTRL, &[0x17]),
    InitStep::CmdData(Cmd::SOURCE_VOLTAGE_CTRL, &[0x41, 0x00, 0x32]),
    InitStep::CmdData(Cmd::WRITE_VCOM_REG, &[0x36]),
    InitStep::CmdData(Cmd::WRITE_LUT_REG, &TI_290_MONOFULL_LUT_CODE),
];

/// Initialization sequence for the 2.9in panel 2-bit grayscale
pub const INIT_SEQUENCE_GRAY2: &[InitStep] = &[
    // Set Initialial Configuration
    InitStep::SWReset,
    InitStep::DelayMs(10),
    // Send Initialization Code 0x01, 0x11, 0x44, 0x45, 0x3C
    InitStep::CmdData(Cmd::DRIVER_OUTPUT_CTRL, &[0x27, 0x01, 0x00]),
    InitStep::CmdData(Cmd::DATA_ENTRY_MODE, &[flag::Flag::DATA_ENTRY_INCRY_INCRX]),
    InitStep::CmdData(Cmd::SET_RAMXPOS, &[0x00, 0x0F]),
    InitStep::CmdData(Cmd::SET_RAMYPOS, &[0x00, 0x00, 0x27, 0x01]),
    InitStep::CmdData(
        Cmd::BORDER_WAVEFORM_CTRL,
        &[flag::Flag::BORDER_WAVEFORM_FOLLOW_LUT | flag::Flag::BORDER_WAVEFORM_LUT1],
    ),
    // Load Waveform LUT
    InitStep::CmdData(Cmd::TEMP_CONTROL, &[flag::Flag::INTERNAL_TEMP_SENSOR]),
    InitStep::CmdData(Cmd::DISPLAY_UPDATE_CTRL1, &[0x00, 0x80]),
    // InitStep::CmdData(Cmd::DISPLAY_UPDATE_CTRL2, &[0x20]), // load default LUT (default mono)
    // Used for grayscale LUT (not yet implemented)
    // load LUT into memory instead!
    InitStep::CmdData(Cmd::END_OPTION, &[flag::Flag::END_OPTION_NORMAL]),
    InitStep::CmdData(Cmd::GATE_VOLTAGE_CTRL, &[0x17]),
    InitStep::CmdData(Cmd::SOURCE_VOLTAGE_CTRL, &[0x41, 0x00, 0x32]),
    InitStep::CmdData(Cmd::WRITE_VCOM_REG, &[0x36]),
    InitStep::CmdData(Cmd::WRITE_LUT_REG, &TI_290MFGN_GRAY2_LUT_CODE),
];

#[rustfmt::skip]
/// Adafruit ThinkInk 290 EA4MFGN MONO FULL LUT CODE Cmd: 0x32 Size: 0x99,
pub const TI_290_MONOFULL_LUT_CODE: [u8; 153] = [
  0x80,	0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, //VS L0
  0x10, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, //VS L1
  0x80, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, //VS L2
  0x10, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, //VS L3
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //VS L4
  0x14, 0x08, 0x00, 0x00, 0x00, 0x00, 0x01,                               //TP, SR, RP of Group0
  0x0A, 0x0A, 0x00, 0x0A, 0x0A, 0x00, 0x01,                               //TP, SR, RP of Group1
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group2
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group3
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group4
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group5
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group6
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group7
  0x14, 0x08, 0x00, 0x01, 0x00, 0x00, 0x01,                               //TP, SR, RP of Group8
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,                               //TP, SR, RP of Group9
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group10
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group11
  0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x00, 0x00, 0x00                    //FR, XON
];

#[rustfmt::skip]
/// Adafruit ThinkInk 290 EA4MFGN GRAY2 LUT CODE Cmd: 0x32 Size: 0x99,
pub const TI_290MFGN_GRAY2_LUT_CODE: [u8; 153] = [
  0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //VS L0	 //2.28s
  0x20, 0x60, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //VS L1
  0x28, 0x60, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //VS L2
  0x2A, 0x60, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //VS L3
  0x00, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //VS L4
  0x00, 0x02, 0x00, 0x05, 0x14, 0x00, 0x00,                               //TP, SR, RP of Group0
  0x1E, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x01,                               //TP, SR, RP of Group1
  0x00, 0x02, 0x00, 0x05, 0x14, 0x00, 0x00,                               //TP, SR, RP of Group2
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group3
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group4
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group5
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group6
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group7
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group8
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group9
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group10
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               //TP, SR, RP of Group11
  0x24, 0x22, 0x22, 0x22, 0x23, 0x32, 0x00, 0x00, 0x00,                   //FR, XON
];

/// Display for a 128x296 panel (2.9in) with integrated driver
pub struct ThinkInk2in9Mono<SPI, BSY, DC, RST>
where
    SPI: SpiDevice,
    BSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
{
    interface: SpiDisplayInterface<SPI, BSY, DC, RST>,
}

impl<SPI, BSY, DC, RST> ThinkInk2in9Mono<SPI, BSY, DC, RST>
where
    SPI: SpiDevice,
    BSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
{
    /// Create a new ThinkInk 2.9" display
    pub fn new(spi: SPI, busy: BSY, dc: DC, rst: RST) -> Result<Self, DisplayError> {
        let interface = SpiDisplayInterface::new(spi, busy, dc, rst);
        Ok(Self { interface })
    }

    /// Update the whole BW buffer on the display driver
    fn write_bw_ram(&mut self, buffer: &[u8]) -> Result<(), DisplayError> {
        self.set_ram_counter(0, 0)?;
        self.interface.cmd_with_data(Cmd::WRITE_BW_DATA, buffer)
    }

    /// Update the whole Red buffer on the display driver
    fn write_red_ram(&mut self, buffer: &[u8]) -> Result<(), DisplayError> {
        self.set_ram_counter(0, 0)?;
        self.interface.cmd_with_data(Cmd::WRITE_RED_DATA, buffer)
    }

    fn display(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.interface
            .cmd_with_data(Cmd::DISPLAY_UPDATE_CTRL2, &[flag::Flag::DISPLAY_MODE_1])?;
        self.interface.cmd(Cmd::MASTER_ACTIVATE)?;
        self.interface.wait_until_idle(delay);
        Ok(())
    }

    /// Clear the display
    pub fn clear_display(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.clear_bw_ram()?;
        self.clear_red_ram()?;
        self.display(delay)?;
        delay.delay_ms(100);
        self.display(delay)
    }

    /// Update both black & white
    pub fn update_and_display(
        &mut self,
        bw_buffer: &[u8],
        delay: &mut impl DelayNs,
    ) -> Result<(), DisplayError> {
        self.init(delay)?;
        self.update_bw(bw_buffer, delay)?;
        self.display(delay)?;
        self.sleep(delay)
    }

    /// Update both black & white
    pub fn update_gray2_and_display(
        &mut self,
        bw_buffer: &[u8],
        red_buffer: &[u8],
        delay: &mut impl DelayNs,
    ) -> Result<(), DisplayError> {
        self.init(delay)?;
        self.update_bw(bw_buffer, delay)?;
        self.update_red(red_buffer, delay)?;
        self.display(delay)?;
        self.sleep(delay)
    }

    fn use_full_frame(&mut self) -> Result<(), DisplayError> {
        self.set_ram_area(0, 0, WIDTH - 1, HEIGHT - 1)?;
        self.set_ram_counter(0, 0)
    }

    fn set_ram_area(
        &mut self,
        start_x: u16,
        start_y: u16,
        end_x: u16,
        end_y: u16,
    ) -> Result<(), DisplayError> {
        self.interface.cmd_with_data(
            Cmd::SET_RAMXPOS,
            &[(start_x >> 3) as u8, (end_x >> 3) as u8],
        )?;

        self.interface.cmd_with_data(
            Cmd::SET_RAMYPOS,
            &[
                (start_y & 0xFF) as u8,
                ((start_y >> 8) & 0x01) as u8,
                (end_y & 0xFF) as u8,
                ((end_y >> 8) & 0x01) as u8,
            ],
        )?;
        Ok(())
    }

    fn set_ram_counter(&mut self, x: u32, y: u32) -> Result<(), DisplayError> {
        self.interface
            .cmd_with_data(Cmd::SET_RAMX_COUNTER, &[(x >> 3) as u8])?;

        self.interface.cmd_with_data(
            Cmd::SET_RAMY_COUNTER,
            &[(y & 0xFF) as u8, ((y >> 8) & 0x01) as u8],
        )?;
        Ok(())
    }
}

impl<SPI, BSY, DC, RST> Ssd1680Driver for ThinkInk2in9Mono<SPI, BSY, DC, RST>
where
    SPI: SpiDevice,
    BSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
{
    fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        debug!("powering up ThinkInk 2.9\" display");

        // hard reset
        self.interface.hard_reset(delay)?;

        for step in INIT_SEQUENCE_MONOFULL {
            debug!("init step: {:?}", step);
            match *step {
                InitStep::SWReset => {
                    self.interface.cmd(Cmd::SW_RESET)?;
                }
                InitStep::DelayMs(ms) => {
                    delay.delay_ms(u32::from(ms));
                }
                InitStep::BusyWait => {
                    self.interface.wait_until_idle(delay);
                }
                InitStep::Cmd(c) => {
                    self.interface.cmd(c)?;
                }
                InitStep::CmdData(c, d) => {
                    self.interface.cmd_with_data(c, d)?;
                }
                InitStep::UseFullFrame => {
                    self.use_full_frame()?;
                }
            }
        }

        self.interface.wait_until_idle(delay);
        Ok(())
    }

    fn sleep(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        debug!("powering down ThinkInk 2.9\" display");
        self.interface.cmd_with_data(Cmd::DEEP_SLEEP, &[0x01])?;
        delay.delay_ms(1);
        Ok(())
    }

    fn update_bw(&mut self, buffer: &[u8], delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.write_bw_ram(buffer)?;
        self.interface.wait_until_idle(delay);
        Ok(())
    }

    fn update_red(&mut self, buffer: &[u8], delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.write_red_ram(buffer)?;
        self.interface.wait_until_idle(delay);
        Ok(())
    }

    fn update(
        &mut self,
        low_buffer: &[u8],
        high_buffer: &[u8],
        delay: &mut impl DelayNs,
    ) -> Result<(), DisplayError> {
        self.write_bw_ram(low_buffer)?;
        self.write_red_ram(high_buffer)?;
        self.interface.wait_until_idle(delay);
        Ok(())
    }

    fn clear_bw_ram(&mut self) -> Result<(), DisplayError> {
        let color = Color::White.get_byte_value();
        self.interface.cmd(Cmd::WRITE_BW_DATA)?;
        self.interface
            .data_x_times(color, u32::from(WIDTH).div_ceil(8) * u32::from(HEIGHT))?;
        Ok(())
    }

    fn clear_red_ram(&mut self) -> Result<(), DisplayError> {
        let color = Color::White.inverse().get_byte_value();
        self.interface.cmd(Cmd::WRITE_RED_DATA)?;
        self.interface
            .data_x_times(color, u32::from(WIDTH).div_ceil(8) * u32::from(HEIGHT))?;
        Ok(())
    }

    fn begin(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.interface.hard_reset(delay)?;
        self.sleep(delay)
    }
}

/// Variant of the 2.9" ThinkInk display that uses the 2-bit grayscale init sequence
pub struct ThinkInk2in9Gray2<SPI, BSY, DC, RST>
where
    SPI: SpiDevice,
    BSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
{
    interface: SpiDisplayInterface<SPI, BSY, DC, RST>,
}

impl<SPI, BSY, DC, RST> ThinkInk2in9Gray2<SPI, BSY, DC, RST>
where
    SPI: SpiDevice,
    BSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
{
    /// Create a new ThinkInk 2.9" grayscale display
    pub fn new(spi: SPI, busy: BSY, dc: DC, rst: RST) -> Result<Self, DisplayError> {
        let interface = SpiDisplayInterface::new(spi, busy, dc, rst);
        Ok(Self { interface })
    }

    /// Update the whole BW buffer on the display driver
    fn write_bw_ram(&mut self, buffer: &[u8]) -> Result<(), DisplayError> {
        self.set_ram_counter(0, 0)?;
        self.interface.cmd_with_data(Cmd::WRITE_BW_DATA, buffer)
    }

    /// Update the whole Red buffer on the display driver
    fn write_red_ram(&mut self, buffer: &[u8]) -> Result<(), DisplayError> {
        self.set_ram_counter(0, 0)?;
        self.interface.cmd_with_data(Cmd::WRITE_RED_DATA, buffer)
    }

    fn display(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.interface
            .cmd_with_data(Cmd::DISPLAY_UPDATE_CTRL2, &[flag::Flag::DISPLAY_MODE_1])?;
        self.interface.cmd(Cmd::MASTER_ACTIVATE)?;
        self.interface.wait_until_idle(delay);
        Ok(())
    }

    /// Clear the display
    pub fn clear_display(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.clear_bw_ram()?;
        self.clear_red_ram()?;
        self.display(delay)?;
        delay.delay_ms(100);
        self.display(delay)
    }

    /// Update both black & white
    pub fn update_and_display(
        &mut self,
        bw_buffer: &[u8],
        delay: &mut impl DelayNs,
    ) -> Result<(), DisplayError> {
        self.init(delay)?;
        self.update_bw(bw_buffer, delay)?;
        self.display(delay)?;
        self.sleep(delay)
    }

    /// Update both black & white
    pub fn update_gray2_and_display(
        &mut self,
        high_buffer: &[u8],
        low_buffer: &[u8],
        delay: &mut impl DelayNs,
    ) -> Result<(), DisplayError> {
        self.init(delay)?;
        self.update_bw(high_buffer, delay)?;
        self.update_red(low_buffer, delay)?;
        self.display(delay)?;
        self.sleep(delay)
    }

    fn use_full_frame(&mut self) -> Result<(), DisplayError> {
        self.set_ram_area(0, 0, WIDTH - 1, HEIGHT - 1)?;
        self.set_ram_counter(0, 0)
    }

    fn set_ram_area(
        &mut self,
        start_x: u16,
        start_y: u16,
        end_x: u16,
        end_y: u16,
    ) -> Result<(), DisplayError> {
        self.interface.cmd_with_data(
            Cmd::SET_RAMXPOS,
            &[(start_x >> 3) as u8, (end_x >> 3) as u8],
        )?;

        self.interface.cmd_with_data(
            Cmd::SET_RAMYPOS,
            &[
                (start_y & 0xFF) as u8,
                ((start_y >> 8) & 0x01) as u8,
                (end_y & 0xFF) as u8,
                ((end_y >> 8) & 0x01) as u8,
            ],
        )?;
        Ok(())
    }

    fn set_ram_counter(&mut self, x: u32, y: u32) -> Result<(), DisplayError> {
        self.interface
            .cmd_with_data(Cmd::SET_RAMX_COUNTER, &[(x >> 3) as u8])?;

        self.interface.cmd_with_data(
            Cmd::SET_RAMY_COUNTER,
            &[(y & 0xFF) as u8, ((y >> 8) & 0x01) as u8],
        )?;
        Ok(())
    }
}

impl<SPI, BSY, DC, RST> Ssd1680Driver for ThinkInk2in9Gray2<SPI, BSY, DC, RST>
where
    SPI: SpiDevice,
    BSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
{
    fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        debug!("powering up ThinkInk 2.9\" grayscale display");

        // hard reset
        self.interface.hard_reset(delay)?;

        for step in INIT_SEQUENCE_GRAY2 {
            debug!("init step: {:?}", step);
            match *step {
                InitStep::SWReset => {
                    self.interface.cmd(Cmd::SW_RESET)?;
                }
                InitStep::DelayMs(ms) => {
                    delay.delay_ms(u32::from(ms));
                }
                InitStep::BusyWait => {
                    self.interface.wait_until_idle(delay);
                }
                InitStep::Cmd(c) => {
                    self.interface.cmd(c)?;
                }
                InitStep::CmdData(c, d) => {
                    self.interface.cmd_with_data(c, d)?;
                }
                InitStep::UseFullFrame => {
                    self.use_full_frame()?;
                }
            }
        }

        self.interface.wait_until_idle(delay);
        Ok(())
    }

    fn sleep(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        debug!("powering down ThinkInk 2.9\" grayscale display");
        self.interface.cmd_with_data(Cmd::DEEP_SLEEP, &[0x01])?;
        delay.delay_ms(1);
        Ok(())
    }

    fn update_bw(&mut self, buffer: &[u8], delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.write_bw_ram(buffer)?;
        self.interface.wait_until_idle(delay);
        Ok(())
    }

    fn update_red(&mut self, buffer: &[u8], delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.write_red_ram(buffer)?;
        self.interface.wait_until_idle(delay);
        Ok(())
    }

    fn update(
        &mut self,
        bw_buffer: &[u8],
        red_buffer: &[u8],
        delay: &mut impl DelayNs,
    ) -> Result<(), DisplayError> {
        self.write_bw_ram(bw_buffer)?;
        self.write_red_ram(red_buffer)?;
        self.interface.wait_until_idle(delay);
        Ok(())
    }

    fn clear_bw_ram(&mut self) -> Result<(), DisplayError> {
        let color = Color::White.get_byte_value();
        self.interface.cmd(Cmd::WRITE_BW_DATA)?;
        self.interface
            .data_x_times(color, u32::from(WIDTH).div_ceil(8) * u32::from(HEIGHT))?;
        Ok(())
    }

    fn clear_red_ram(&mut self) -> Result<(), DisplayError> {
        let color = Color::White.inverse().get_byte_value();
        self.interface.cmd(Cmd::WRITE_RED_DATA)?;
        self.interface
            .data_x_times(color, u32::from(WIDTH).div_ceil(8) * u32::from(HEIGHT))?;
        Ok(())
    }

    fn begin(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.interface.hard_reset(delay)?;
        self.sleep(delay)
    }
}

/// Graphics buffer for the 2.9" display
pub struct Display2in9 {
    buffer: [u8; buffer_len(WIDTH as usize, HEIGHT as usize)],
    rotation: DisplayRotation,
    is_inverted: bool,
}

impl Default for Display2in9 {
    fn default() -> Self {
        Self::new()
    }
}

impl Display2in9 {
    /// Create a new black and white graphics buffer for the 2.9" display
    pub fn new() -> Self {
        Self {
            buffer: [Color::White.get_byte_value(); buffer_len(WIDTH as usize, HEIGHT as usize)],
            rotation: DisplayRotation::Rotate270,
            is_inverted: false,
        }
    }

    /// Display the current buffer contents on the given driver
    pub fn update_bw<SPI, BSY, DC, RST>(
        &self,
        driver: &mut ThinkInk2in9Mono<SPI, BSY, DC, RST>,
        delay: &mut impl DelayNs,
    ) -> Result<(), DisplayError>
    where
        SPI: SpiDevice,
        BSY: InputPin,
        DC: OutputPin,
        RST: OutputPin,
    {
        driver.update_bw(&self.buffer, delay)
    }
}

impl DrawTarget for Display2in9 {
    type Error = DisplayError;
    type Color = BinaryColor;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for p in pixels.into_iter() {
            self.draw_helper(WIDTH.into(), HEIGHT.into(), p)?;
        }
        Ok(())
    }
}

impl OriginDimensions for Display2in9 {
    fn size(&self) -> Size {
        //if display is rotated 90 deg or 270 then swap height and width
        match self.rotation() {
            DisplayRotation::Rotate0 | DisplayRotation::Rotate180 => {
                Size::new(WIDTH.into(), HEIGHT.into())
            }
            DisplayRotation::Rotate90 | DisplayRotation::Rotate270 => {
                Size::new(HEIGHT.into(), WIDTH.into())
            }
        }
    }
}

impl Display for Display2in9 {
    fn buffer(&self) -> &[u8] {
        &self.buffer
    }

    fn get_mut_buffer(&mut self) -> &mut [u8] {
        &mut self.buffer
    }

    fn set_rotation(&mut self, rotation: DisplayRotation) {
        self.rotation = rotation;
    }

    fn rotation(&self) -> DisplayRotation {
        self.rotation
    }

    fn is_inverted(&self) -> bool {
        self.is_inverted
    }
}

/// Grayscale (2-bit) graphics buffer for the 2.9" display
///
/// This type holds two internal framebuffers (bw + red) which together encode
/// 2-bit grayscale per pixel. The logical mapping (bw_bit, red_bit) is:
/// - (1,1) = White
/// - (1,0) = Light gray
/// - (0,1) = Dark gray
/// - (0,0) = Black
///
/// If `inverted` is true (default) the stored bits are inverted to match the
/// display's expected polarity in grayscale mode. Callers should obtain the
/// buffers with `low_buffer()` and `high_buffer()` and pass them to the driver
/// `update_gray2_and_display` method.
pub struct Display2in9Gray2 {
    high_buffer: [u8; buffer_len(WIDTH as usize, HEIGHT as usize)],
    low_buffer: [u8; buffer_len(WIDTH as usize, HEIGHT as usize)],
    rotation: DisplayRotation,
    inverted: bool,
}

// Use embedded-graphics Gray2 for grayscale levels

impl Display2in9Gray2 {
    /// Create a new grayscale buffer. By default the stored buffers are
    /// inverted to match the display polarity for grayscale mode.
    pub fn new() -> Self {
        // When inverted is true we initialize buffers with 0x00 which corresponds
        // to logical white when the polarity is inverted. This matches other
        // display buffer initializations in this crate.
        Self {
            high_buffer: [Color::White.inverse().get_byte_value();
                buffer_len(WIDTH as usize, HEIGHT as usize)],
            low_buffer: [Color::White.inverse().get_byte_value();
                buffer_len(WIDTH as usize, HEIGHT as usize)],
            rotation: DisplayRotation::Rotate270,
            inverted: true,
        }
    }

    /// Get a reference to the low buffer (first plane)
    pub fn low_buffer(&self) -> &[u8] {
        &self.low_buffer
    }

    /// Get a reference to the high buffer (second plane)
    pub fn high_buffer(&self) -> &[u8] {
        &self.high_buffer
    }

    /// Get mutable access to the low buffer
    pub fn get_mut_low_buffer(&mut self) -> &mut [u8] {
        &mut self.low_buffer
    }

    /// Get mutable access to the high buffer
    pub fn get_mut_high_buffer(&mut self) -> &mut [u8] {
        &mut self.high_buffer
    }

    /// Set the rotation used for coordinate transforms
    pub fn set_rotation(&mut self, rotation: DisplayRotation) {
        self.rotation = rotation;
    }

    /// Get the current rotation
    pub fn rotation(&self) -> DisplayRotation {
        self.rotation
    }

    /// Whether the stored buffers are inverted
    pub fn is_inverted(&self) -> bool {
        self.inverted
    }

    /// Clear both buffers to the given grayscale level
    pub fn clear_buffer(&mut self, level: Gray2) {
        // Gray2::into_storage() gives 0..3 where bits are [bw, red]
        let storage = level.into_storage();
        let bw_bit = (storage & 0b10) != 0;
        let red_bit = (storage & 0b01) != 0;

        // compute the byte value for the bit (msb first)
        let bw_byte = if bw_bit { 0xFF } else { 0x00 };
        let red_byte = if red_bit { 0xFF } else { 0x00 };

        // apply inversion if necessary (stored polarity is inverted)
        let bw_stored = if self.inverted { !bw_byte } else { bw_byte };
        let red_stored = if self.inverted { !red_byte } else { red_byte };

        for b in self.high_buffer.iter_mut() {
            *b = bw_stored;
        }
        for b in self.low_buffer.iter_mut() {
            *b = red_stored;
        }
    }

    /// Set a single pixel to the given 2-bit grayscale level
    pub fn set_pixel(&mut self, x: i32, y: i32, level: Gray2) {
        if super::outside_display(Point::new(x, y), WIDTH.into(), HEIGHT.into(), self.rotation) {
            return;
        }
        let (idx_u32, bit) = super::find_position(
            x as u32,
            y as u32,
            WIDTH.into(),
            HEIGHT.into(),
            self.rotation,
        );
        let idx = idx_u32 as usize;

        // Inline Gray2 bit decoding: storage bits are [bw, red]
        let storage = level.into_storage();
        let bw_bit = (storage & 0b10) != 0;
        let red_bit = (storage & 0b01) != 0;

        // Determine stored bit values accounting for inversion
        let bw_val = if self.inverted { !bw_bit } else { bw_bit };
        let red_val = if self.inverted { !red_bit } else { red_bit };

        if bw_val {
            self.high_buffer[idx] |= bit;
        } else {
            self.high_buffer[idx] &= !bit;
        }

        if red_val {
            self.low_buffer[idx] |= bit;
        } else {
            self.low_buffer[idx] &= !bit;
        }
    }
}

impl Default for Display2in9Gray2 {
    fn default() -> Self {
        Self::new()
    }
}

impl DrawTarget for Display2in9Gray2 {
    type Error = DisplayError;
    type Color = Gray2;

    /// Draw binary pixels: BinaryColor::On is treated as White, Off as Black.
    /// This provides basic compatibility with embedded-graphics for binary content.
    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for p in pixels.into_iter() {
            let Pixel(point, color) = p;
            self.set_pixel(point.x, point.y, color);
        }
        Ok(())
    }
}

/// Adapter that exposes a BinaryColor DrawTarget view over a Gray2 display buffer.
pub struct BinaryDrawTarget<'a>(&'a mut Display2in9Gray2);

impl<'a> BinaryDrawTarget<'a> {
    /// Create a new BinaryDrawTarget adapter
    pub fn new(display: &'a mut Display2in9Gray2) -> Self {
        Self(display)
    }
}

impl<'a> DrawTarget for BinaryDrawTarget<'a> {
    type Color = BinaryColor;
    type Error = DisplayError;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            let level = match color {
                BinaryColor::On => Gray2::WHITE,
                BinaryColor::Off => Gray2::BLACK,
            };
            self.0.set_pixel(point.x, point.y, level);
        }
        Ok(())
    }
}

impl OriginDimensions for BinaryDrawTarget<'_> {
    fn size(&self) -> Size {
        match self.0.rotation {
            DisplayRotation::Rotate0 | DisplayRotation::Rotate180 => {
                Size::new(WIDTH.into(), HEIGHT.into())
            }
            DisplayRotation::Rotate90 | DisplayRotation::Rotate270 => {
                Size::new(HEIGHT.into(), WIDTH.into())
            }
        }
    }
}

impl Display2in9Gray2 {
    /// Get an adapter that implements DrawTarget<BinaryColor>.
    pub fn as_binary_draw_target(&mut self) -> BinaryDrawTarget<'_> {
        BinaryDrawTarget::new(self)
    }
}

impl OriginDimensions for Display2in9Gray2 {
    fn size(&self) -> Size {
        match self.rotation {
            DisplayRotation::Rotate0 | DisplayRotation::Rotate180 => {
                Size::new(WIDTH.into(), HEIGHT.into())
            }
            DisplayRotation::Rotate90 | DisplayRotation::Rotate270 => {
                Size::new(HEIGHT.into(), WIDTH.into())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Display, Display2in9, DisplayRotation, HEIGHT, WIDTH};
    use crate::color::{Black, Color};
    use crate::displays::{find_position, outside_display};
    use embedded_graphics::{prelude::*, primitives::Line, primitives::PrimitiveStyle};

    #[test]
    fn buffer_clear() {
        let mut display = Display2in9::new();

        for &byte in display.buffer().iter() {
            assert_eq!(byte, Color::White.get_byte_value());
        }

        display.clear_buffer(Color::Black);

        for &byte in display.buffer().iter() {
            assert_eq!(byte, Color::Black.get_byte_value());
        }
    }

    #[test]
    fn rotation_overflow() {
        let width = WIDTH as u32;
        let height = HEIGHT as u32;
        test_rotation_overflow(width, height, DisplayRotation::Rotate0);
        test_rotation_overflow(width, height, DisplayRotation::Rotate90);
        test_rotation_overflow(width, height, DisplayRotation::Rotate180);
        test_rotation_overflow(width, height, DisplayRotation::Rotate270);
    }

    fn test_rotation_overflow(width: u32, height: u32, rotation: DisplayRotation) {
        let max_value = width.div_ceil(8) * height;
        for x in 0..(width + height) {
            for y in 0..u32::MAX {
                if outside_display(Point::new(x as i32, y as i32), width, height, rotation) {
                    break;
                } else {
                    let (idx, _) = find_position(x, y, width, height, rotation);
                    assert!(idx < max_value, "{idx} !< {max_value}",);
                }
            }
        }
    }

    #[test]
    fn graphics_rotation_0() {
        let mut display = Display2in9::new();
        display.set_rotation(DisplayRotation::Rotate0);

        let _ = Line::new(Point::new(0, 0), Point::new(7, 0))
            .into_styled(PrimitiveStyle::with_stroke(Black, 1))
            .draw(&mut display);

        let buffer = display.buffer();
        assert_eq!(buffer[0], Color::Black.get_byte_value());

        for &byte in buffer.iter().skip(1) {
            assert_eq!(byte, Color::White.get_byte_value());
        }
    }
}
