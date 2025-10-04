use super::DisplayRotation;
use super::{buffer_len, Display};
use crate::color::Color;
use crate::{cmd, driver, flag, make_new_display_driver};
use display_interface::DisplayError;
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};

/// Display width for 2.9in display
pub const WIDTH: u16 = 128;
/// Display height for 2.9in display
pub const HEIGHT: u16 = 296;

/// Initialization sequence for the 2.9in panel. Can be passed to
/// `driver::Ssd1680::from_spi_with_init_sequence`.
pub const INIT_SEQUENCE: &[driver::InitStep] = &[
    driver::InitStep::SWReset,
    driver::InitStep::DelayMs(10),
    driver::InitStep::WaitUntilIdle,
    driver::InitStep::DriverControl,
    driver::InitStep::CmdData(
        cmd::Cmd::DATA_ENTRY_MODE,
        &[flag::Flag::DATA_ENTRY_INCRY_INCRX],
    ),
    driver::InitStep::CmdData(
        cmd::Cmd::BORDER_WAVEFORM_CTRL,
        &[flag::Flag::BORDER_WAVEFORM_FOLLOW_LUT | flag::Flag::BORDER_WAVEFORM_LUT1],
    ),
    driver::InitStep::CmdData(cmd::Cmd::TEMP_CONTROL, &[flag::Flag::INTERNAL_TEMP_SENSOR]),
    driver::InitStep::CmdData(cmd::Cmd::DISPLAY_UPDATE_CTRL1, &[0x00, 0x80]),
    driver::InitStep::UseFullFrame,
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

// Create a SPI-backed `Ssd1680` driver pre-configured for this panel.
make_new_display_driver!();

/// Display for a 129x296 panel (2.9in rotated)
pub struct ThinkInkMono2in9 {
    buffer: [u8; buffer_len(WIDTH as usize, HEIGHT as usize)],
    rotation: DisplayRotation,
    is_inverted: bool,
}

impl Default for ThinkInkMono2in9 {
    /// Create a black & white buffer for the 2.9in display
    fn default() -> Self {
        ThinkInkMono2in9 {
            buffer: [Color::White.get_byte_value(); buffer_len(WIDTH as usize, HEIGHT as usize)],
            rotation: DisplayRotation::Rotate270,
            is_inverted: false,
        }
    }
}

impl DrawTarget for ThinkInkMono2in9 {
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

impl OriginDimensions for ThinkInkMono2in9 {
    fn size(&self) -> Size {
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

impl Display for ThinkInkMono2in9 {
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
    use super::{Display, DisplayRotation, ThinkInkMono2in9};
    use crate::color::Black;
    use crate::color::Color;
    use crate::displays::find_position;
    use crate::displays::outside_display;
    use embedded_graphics::{prelude::*, primitives::Line, primitives::PrimitiveStyle};

    #[test]
    fn buffer_clear() {
        let mut display = ThinkInkMono2in9::default();

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
        use super::{HEIGHT, WIDTH};
        let width = WIDTH as u32;
        let height = HEIGHT as u32;
        test_rotation_overflow(width, height, DisplayRotation::Rotate0);
        test_rotation_overflow(width, height, DisplayRotation::Rotate90);
        test_rotation_overflow(width, height, DisplayRotation::Rotate180);
        test_rotation_overflow(width, height, DisplayRotation::Rotate270);
    }

    fn test_rotation_overflow(width: u32, height: u32, rotation2: DisplayRotation) {
        let max_value = width.div_ceil(8) * height;
        for x in 0..(width + height) {
            //limit x because it runs too long
            for y in 0..u32::MAX {
                if outside_display(Point::new(x as i32, y as i32), width, height, rotation2) {
                    break;
                } else {
                    let (idx, _) = find_position(x, y, width, height, rotation2);
                    assert!(idx < max_value, "{idx} !< {max_value}",);
                }
            }
        }
    }

    #[test]
    fn graphics_rotation_0() {
        let mut display = ThinkInkMono2in9::default();
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
