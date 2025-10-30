//! SSD1680 ePaper Display Driver
//!
//! Used in the [WeAct 2.13" Tri-Color display](https://www.aliexpress.com/item/1005004644515880.html)
//! or [Adafruit ThinkInk 2.9" Mono / 4 Grayscale display](https://www.adafruit.com/product/4777)
//!
//! For a complete example see [the example](https://github.com/mbv/esp32-ssd1680/blob/main/src/main.rs).
//!
//! This driver is loosely modeled after the
//! [epd-waveshare](https://github.com/caemor/epd-waveshare) drivers but built for my needs.
//!
//! ## Architecture
//!
//! This driver separates hardware control from graphics rendering:
//! - **Driver structs** (`WeActStudio2in13`, `ThinkInk2in9Mono`) handle hardware interface and SSD1680 commands
//! - **Graphics structs** (`Display2in13`, `Display2in9`) handle frame buffers and embedded-graphics integration
//!
//! This allows multiple graphics buffers (e.g., for different color layers) to share a single hardware driver.
//!
//! ## Usage
//!
//! ### WeAct Studio 2.13" Display (Tri-Color)
//!
//! ```rust, ignore
//! use ssd1680::displays::weact_studio_2in13::{WeActStudio2in13, Display2in13};
//! use embedded_graphics::{prelude::*, primitives::*, mono_font::{ascii::FONT_6X10, MonoTextStyle}};
//!
//! // 1. Create the hardware driver
//! let mut driver = WeActStudio2in13::new(spi, busy, dc, rst)?;
//!
//! // 2. Create graphics buffers
//! let mut bw_display = Display2in13::bw();   // Black & white layer
//! let mut red_display = Display2in13::red(); // Red layer
//!
//! // 3. Draw to the buffers using embedded-graphics
//! Rectangle::new(Point::new(10, 10), Size::new(50, 30))
//!     .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
//!     .draw(&mut bw_display)?;
//!
//! Circle::new(Point::new(60, 20), 25)
//!     .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
//!     .draw(&mut red_display)?;
//!
//! // 4. Update display with both buffers in one command
//! driver.update_and_display(bw_display.buffer(), red_display.buffer(), &mut delay)?;
//! ```
//!
//! ### Adafruit ThinkInk 2.9" Display (Mono)
//!
//! ```rust, ignore
//! use ssd1680::displays::adafruit_thinkink_2in9::{ThinkInk2in9Mono, Display2in9};
//!
//! // Create driver and graphics buffer
//! let mut driver = ThinkInk2in9Mono::new(spi, busy, dc, rst)?;
//! let mut display = Display2in9::new();
//!
//! // Draw and update in one command
//! driver.update_and_display(display.buffer(), &mut delay)?;
//! ```
//!
//! For advanced use cases, you can also use the individual `update_bw()`, `update_red()`,
//! `update()`, and `update_display()` methods for more granular control.
//!
#![no_std]
#![deny(missing_docs)]
#![allow(clippy::pedantic)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]
#![allow(clippy::must_use_candidate)]
#![allow(clippy::missing_errors_doc)]

mod cmd;
pub mod color;
#[cfg(feature = "graphics")]
pub mod displays;
pub mod driver;

mod flag;

/// Maximum display height this driver supports
pub const MAX_HEIGHT: u16 = 296;

/// Maximum display width this driver supports
pub const MAX_WIDTH: u16 = 176;

pub mod interface;

/// Useful exports
pub mod prelude {
    pub use crate::color::Color;
    pub use crate::driver::Ssd1680Driver;

    #[cfg(feature = "graphics")]
    pub use crate::displays::{Display, DisplayRotation};
}
