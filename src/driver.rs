//! Driver for interacting with EPD displays
pub use display_interface::DisplayError;

use embedded_hal::delay::DelayNs;

/// Trait defining the driver interface for SSD1680 displays
pub trait Ssd1680Driver {
    /// Reset and initialize the display
    fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError>;

    /// Power down the controller
    fn sleep(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError>;

    /// Send black/white buffer to display
    fn update_bw(&mut self, buffer: &[u8], delay: &mut impl DelayNs) -> Result<(), DisplayError>;

    /// Send red buffer
    fn update_red(&mut self, buffer: &[u8], delay: &mut impl DelayNs) -> Result<(), DisplayError>;

    /// Send both black/white and red buffers to display
    fn update(
        &mut self,
        bw_buffer: &[u8],
        red_buffer: &[u8],
        delay: &mut impl DelayNs,
    ) -> Result<(), DisplayError>;

    /// Clear the BW RAM
    fn clear_bw_ram(&mut self) -> Result<(), DisplayError>;

    /// Clear the red RAM
    fn clear_red_ram(&mut self) -> Result<(), DisplayError>;

    /// Begin operation - reset and put into low power state
    fn begin(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError>;
}
