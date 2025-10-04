//! Graphics Support for EPDs

use crate::color::Color;
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};

/// 2.9in Mono Black/White
pub mod adafruit_thinkink_2in9;
/// 2.13in Tri-Color Black/White/Red
pub mod weact_studio_2in13;

/// Steps that a display-specific init sequence can contain.
/// Keep variants minimal and serializable as static arrays in display modules.
#[derive(Clone, Copy, Debug)]
pub enum InitStep {
    /// Perform a hardware reset (toggle RST pin)
    SWReset,
    /// Wait until the display indicates it's idle/busy line
    DelayMs(u8),
    /// Wait until the display indicates it's idle/busy line
    WaitUntilIdle,
    /// Send a bare command byte
    Cmd(u8),
    /// Send a command with a static data slice
    CmdData(u8, &'static [u8]),
    /// Send DRIVER_OUTPUT_CTRL with [height-1, 0x00, 0x00]
    DriverControl,
    /// Configure display to use the full frame (set ram area/counter)
    UseFullFrame,
}

/// Necessary traits for all displays to implement for drawing
///
/// Adds support for:
/// - Drawing (With tdeploywarselp of DrawTarget/Embedded Graphics)
/// - Rotations
/// - Clearing
pub trait Display: DrawTarget {
    /// Clears the buffer of the display with the chosen background color
    fn clear_buffer(&mut self, background_color: Color) {
        let fill_color = if self.is_inverted() {
            background_color.inverse().get_byte_value()
        } else {
            background_color.get_byte_value()
        };

        for elem in self.get_mut_buffer().iter_mut() {
            *elem = fill_color
        }
    }

    /// Returns the buffer
    fn buffer(&self) -> &[u8];

    /// Returns a mutable buffer
    fn get_mut_buffer(&mut self) -> &mut [u8];

    /// Sets the rotation of the display
    fn set_rotation(&mut self, rotation: DisplayRotation);

    /// Get the current rotation of the display
    fn rotation(&self) -> DisplayRotation;

    /// If the color for this display is inverted
    fn is_inverted(&self) -> bool;

    /// Helperfunction for the Embedded Graphics draw trait
    ///
    /// Becomes uneccesary when `const_generics` become stablised
    fn draw_helper(
        &mut self,
        width: u32,
        height: u32,
        pixel: Pixel<BinaryColor>,
    ) -> Result<(), Self::Error> {
        let rotation = self.rotation();
        let is_inverted = self.is_inverted();
        let buffer = self.get_mut_buffer();

        let Pixel(point, color) = pixel;
        if outside_display(point, width, height, rotation) {
            return Ok(());
        }

        // Give us index inside the buffer and the bit-position in that u8 which needs to be changed
        let (index, bit) = find_position(point.x as u32, point.y as u32, width, height, rotation);
        let index = index as usize;

        // "Draw" the Pixel on that bit
        match color {
            // White/Red
            BinaryColor::On => {
                if is_inverted {
                    buffer[index] &= !bit;
                } else {
                    buffer[index] |= bit;
                }
            }
            //Black
            BinaryColor::Off => {
                if is_inverted {
                    buffer[index] |= bit;
                } else {
                    buffer[index] &= !bit;
                }
            }
        }
        Ok(())
    }
}

/// Displayrotation
#[derive(Clone, Copy, Default)]
pub enum DisplayRotation {
    /// No rotation
    #[default]
    Rotate0,
    /// Rotate by 90 degrees clockwise
    Rotate90,
    /// Rotate by 180 degrees clockwise
    Rotate180,
    /// Rotate 270 degrees clockwise
    Rotate270,
}

// Checks if a pos is outside the defined display
fn outside_display(p: Point, width: u32, height: u32, rotation: DisplayRotation) -> bool {
    if p.x < 0 || p.y < 0 {
        return true;
    }
    let (x, y) = (p.x as u32, p.y as u32);
    match rotation {
        DisplayRotation::Rotate0 | DisplayRotation::Rotate180 => {
            if x >= width || y >= height {
                return true;
            }
        }
        DisplayRotation::Rotate90 | DisplayRotation::Rotate270 => {
            if y >= width || x >= height {
                return true;
            }
        }
    }
    false
}

fn find_rotation(x: u32, y: u32, width: u32, height: u32, rotation: DisplayRotation) -> (u32, u32) {
    let nx;
    let ny;
    match rotation {
        DisplayRotation::Rotate0 => {
            nx = x;
            ny = y;
        }
        DisplayRotation::Rotate90 => {
            nx = width - 1 - y;
            ny = x;
        }
        DisplayRotation::Rotate180 => {
            nx = width - 1 - x;
            ny = height - 1 - y;
        }
        DisplayRotation::Rotate270 => {
            nx = y;
            ny = height - 1 - x;
        }
    }
    (nx, ny)
}

#[rustfmt::skip]
//returns index position in the u8-slice and the bit-position inside that u8
fn find_position(x: u32, y: u32, width: u32, height: u32, rotation: DisplayRotation) -> (u32, u8) {
    let (nx, ny) = find_rotation(x, y, width, height, rotation);
    (
        nx / 8 + width.div_ceil(8) * ny,
        0x80 >> (nx % 8)
    )
}

/// Computes the needed buffer length. Takes care of rounding up in case width
/// is not divisible by 8.
#[must_use]
const fn buffer_len(width: usize, height: usize) -> usize {
    width.div_ceil(8) * height
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;
    use embedded_graphics::pixelcolor::BinaryColor;
    use embedded_graphics::prelude::{Pixel, Point};

    // Simple dummy display with a fixed-size stack buffer. Size chosen large enough for test cases.
    struct DummyDisplay {
        buffer: [u8; 4736],
        rotation: DisplayRotation,
        inverted: bool,
        width: u32,
        height: u32,
    }

    impl DummyDisplay {
        fn new(width: u32, height: u32, rotation: DisplayRotation, inverted: bool) -> Self {
            Self {
                buffer: [0u8; 4736],
                rotation,
                inverted,
                width,
                height,
            }
        }
    }

    impl embedded_graphics::geometry::OriginDimensions for DummyDisplay {
        fn size(&self) -> embedded_graphics::prelude::Size {
            match self.rotation {
                DisplayRotation::Rotate0 | DisplayRotation::Rotate180 => {
                    embedded_graphics::prelude::Size::new(self.width, self.height)
                }
                DisplayRotation::Rotate90 | DisplayRotation::Rotate270 => {
                    embedded_graphics::prelude::Size::new(self.height, self.width)
                }
            }
        }
    }

    impl embedded_graphics::draw_target::DrawTarget for DummyDisplay {
        type Error = Infallible;
        type Color = BinaryColor;

        fn draw_iter<I>(&mut self, _pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>>,
        {
            // Not used in these unit tests.
            for _ in _pixels.into_iter() {}
            Ok(())
        }
    }

    impl Display for DummyDisplay {
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
            self.inverted
        }
    }

    #[test]
    fn draw_helper_sets_msb_first_bits() {
        // width 16 gives two bytes per row
        let mut d = DummyDisplay::new(296, 128, DisplayRotation::Rotate0, false);

        // draw pixels x = 0..7 at y = 0 -> should fill byte 0 with 0xFF (msb-first ordering)
        for x in 0..8 {
            let p = Pixel(Point::new(x, 0), BinaryColor::On);
            let w = d.width;
            let h = d.height;
            Display::draw_helper(&mut d, w, h, p).unwrap();
        }

        assert_eq!(d.buffer[0], 0xFF);

        // other bytes should remain zero
        for &b in d.buffer.iter().skip(1) {
            assert_eq!(b, 0);
        }
    }

    #[test]
    fn draw_helper_sets_last_byte_in_buffer() {
        let width = 296u32;
        let height = 128u32;
        let mut d = DummyDisplay::new(width, height, DisplayRotation::Rotate0, false);

        // draw the bottom-right most pixel
        let x = width - 1;
        let y = height - 1;
        Display::draw_helper(
            &mut d,
            width,
            height,
            Pixel(Point::new(x as i32, y as i32), BinaryColor::On),
        )
        .unwrap();

        let bytes_per_row = width.div_ceil(8);
        let idx = (x / 8 + bytes_per_row * y) as usize;
        let bit = 0x80u8 >> (x % 8);

        // the bit in the last byte should be set
        assert_eq!(d.buffer[idx] & bit, bit);

        // and this should be the final byte in the buffer
        assert_eq!(idx, (bytes_per_row * height - 1) as usize);
    }

    #[test]
    fn draw_helper_respects_rotation() {
        let mut d = DummyDisplay::new(296, 128, DisplayRotation::Rotate90, false);

        // logical point (1,1) under Rotate90 should map to physical (nx,ny)
        let logical_x = 1u32;
        let logical_y = 1u32;

        let w = d.width;
        let h = d.height;
        Display::draw_helper(
            &mut d,
            w,
            h,
            Pixel(
                Point::new(logical_x as i32, logical_y as i32),
                BinaryColor::On,
            ),
        )
        .unwrap();

        let (nx, ny) = super::find_rotation(
            logical_x,
            logical_y,
            d.width,
            d.height,
            DisplayRotation::Rotate90,
        );
        let bytes_per_row = 296u32.div_ceil(8);
        let idx = (nx / 8 + bytes_per_row * ny) as usize;
        let bit = 0x80u8 >> (nx % 8);

        assert_eq!(d.buffer[idx] & bit, bit);
    }

    #[test]
    fn draw_helper_inversion_behavior() {
        let mut d = DummyDisplay::new(8, 1, DisplayRotation::Rotate0, true);

        // initialize to all ones (inverted display uses cleared=0 for 'On')
        for b in d.buffer.iter_mut() {
            *b = 0xFF;
        }

        // draw On -> should clear the bit at x=0
        let w = d.width;
        let h = d.height;
        Display::draw_helper(&mut d, w, h, Pixel(Point::new(0, 0), BinaryColor::On)).unwrap();
        assert_eq!(d.buffer[0] & 0x80, 0);

        // draw Off -> on inverted display should set the bit at x=1 (0x40)
        let w = d.width;
        let h = d.height;
        Display::draw_helper(&mut d, w, h, Pixel(Point::new(1, 0), BinaryColor::Off)).unwrap();
        assert_eq!(d.buffer[0] & 0x40, 0x40);
    }
}
