use super::{buffer_len, Display};
use super::{DisplayRotation, InitStep};
use crate::color::Color;
use crate::{cmd, driver::Ssd1680Driver, flag, interface::SpiDisplayInterface};
use display_interface::DisplayError;
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;
use log::debug;

/// Display width for 2.9in display
pub const WIDTH: u16 = 128;
/// Display height for 2.9in display
pub const HEIGHT: u16 = 296;

/// Initialization sequence for the 2.9in panel
pub const INIT_SEQUENCE: &[InitStep] = &[
    InitStep::SWReset,
    InitStep::DelayMs(10),
    InitStep::WaitUntilIdle,
    InitStep::DriverControl,
    InitStep::CmdData(
        cmd::Cmd::DATA_ENTRY_MODE,
        &[flag::Flag::DATA_ENTRY_INCRY_INCRX],
    ),
    InitStep::CmdData(
        cmd::Cmd::BORDER_WAVEFORM_CTRL,
        &[flag::Flag::BORDER_WAVEFORM_FOLLOW_LUT | flag::Flag::BORDER_WAVEFORM_LUT1],
    ),
    InitStep::CmdData(cmd::Cmd::TEMP_CONTROL, &[flag::Flag::INTERNAL_TEMP_SENSOR]),
    InitStep::CmdData(cmd::Cmd::DISPLAY_UPDATE_CTRL1, &[0x00, 0x80]),
    InitStep::UseFullFrame,
    // driver::InitStep::CmdData(cmd::Cmd::DISPLAY_UPDATE_CTRL2, &[0x20]),
    // driver::InitStep::CmdData(cmd::Cmd::END_OPTION, &[flag::Flag::END_OPTION_NORMAL]),
    // driver::InitStep::CmdData(cmd::Cmd::GATE_VOLTAGE_CTRL, &[0x17]),
    // driver::InitStep::CmdData(cmd::Cmd::SOURCE_VOLTAGE_CTRL, &[0x41, 0x0, 0x32]),
    // driver::InitStep::CmdData(cmd::Cmd::WRITE_VCOM_REG, &[0x36]),
];

#[rustfmt::skip]
/// Adafruit ThinkInk 290 EA4MFGN MONO LUT CODE
pub const TI_290_MONOFULL_LUT_CODE: [u8; 156] = [
  0x32, 0x99,
  0x80,	0x66, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x40, 0x0, 0x0, 0x0,
  0x10, 0x66, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x20, 0x0, 0x0, 0x0,
  0x80, 0x66, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x40, 0x0, 0x0, 0x0,
  0x10, 0x66, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x20, 0x0, 0x0, 0x0,
  0x00, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x00, 0x0, 0x0, 0x0,
  0x14, 0x8, 0x0, 0x0, 0x0, 0x0, 0x1, 				
  0xA, 0xA, 0x0, 0xA, 0xA, 0x0, 0x1, 				
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 				
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 				
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 				
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 				
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 				
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 				
  0x14, 0x8, 0x0, 0x1, 0x0, 0x0, 0x1, 				
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 				
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 				
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 				
  0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x0, 0x0, 0x0, 		
  0xFE
];

/// Display for a 129x296 panel (2.9in) with integrated driver
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
        self.interface
            .cmd_with_data(cmd::Cmd::WRITE_BW_DATA, buffer)
    }

    /// Update the whole Red buffer on the display driver
    fn write_red_ram(&mut self, buffer: &[u8]) -> Result<(), DisplayError> {
        self.set_ram_counter(0, 0)?;
        self.interface
            .cmd_with_data(cmd::Cmd::WRITE_RED_DATA, buffer)
    }

    fn display(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.interface.cmd_with_data(
            cmd::Cmd::DISPLAY_UPDATE_CTRL2,
            &[flag::Flag::DISPLAY_MODE_LOAD_TEMP_1],
        )?;
        self.interface.cmd(cmd::Cmd::MASTER_ACTIVATE)?;
        self.interface.wait_until_idle(delay);
        Ok(())
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
            cmd::Cmd::SET_RAMXPOS,
            &[(start_x >> 3) as u8, (end_x >> 3) as u8],
        )?;

        self.interface.cmd_with_data(
            cmd::Cmd::SET_RAMYPOS,
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
            .cmd_with_data(cmd::Cmd::SET_RAMX_COUNTER, &[(x >> 3) as u8])?;

        self.interface.cmd_with_data(
            cmd::Cmd::SET_RAMY_COUNTER,
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
        self.interface.hard_reset(delay);

        for step in INIT_SEQUENCE {
            debug!("init step: {:?}", step);
            match *step {
                InitStep::SWReset => {
                    self.interface.cmd(cmd::Cmd::SW_RESET)?;
                }
                InitStep::DelayMs(ms) => {
                    delay.delay_ms(u32::from(ms));
                }
                InitStep::WaitUntilIdle => {
                    self.interface.wait_until_idle(delay);
                }
                InitStep::Cmd(c) => {
                    self.interface.cmd(c)?;
                }
                InitStep::CmdData(c, d) => {
                    self.interface.cmd_with_data(c, d)?;
                }
                InitStep::DriverControl => {
                    let h = HEIGHT.saturating_sub(1);
                    let low = (h & 0xFF) as u8;
                    let high = ((h >> 8) & 0x01) as u8;
                    self.interface
                        .cmd_with_data(cmd::Cmd::DRIVER_OUTPUT_CTRL, &[low, high, 0x00])?;
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
        self.interface
            .cmd_with_data(cmd::Cmd::DEEP_SLEEP, &[0x01])?;
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
        self.interface.cmd(cmd::Cmd::WRITE_BW_DATA)?;
        self.interface
            .data_x_times(color, u32::from(WIDTH).div_ceil(8) * u32::from(HEIGHT))?;
        Ok(())
    }

    fn clear_red_ram(&mut self) -> Result<(), DisplayError> {
        let color = Color::White.inverse().get_byte_value();
        self.interface.cmd(cmd::Cmd::WRITE_RED_DATA)?;
        self.interface
            .data_x_times(color, u32::from(WIDTH).div_ceil(8) * u32::from(HEIGHT))?;
        Ok(())
    }

    fn begin(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.interface.hard_reset(delay);
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
