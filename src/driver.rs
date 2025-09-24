//! Driver for interacting with SSD1680 display driver
pub use display_interface::DisplayError;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;

use crate::cmd;
use crate::interface::SpiDisplayInterface;
use crate::{cmd::Cmd, color, flag::Flag};

use log::debug;

// Default sequence keeps previous behavior but is implemented via a small
// array of InitStep so callers can provide their own.
// Mirror the original C default init sequence:
/// Default init sequence if none is provided
pub const DEFAULT_INIT_SEQUENCE: &[InitStep] = &[
    InitStep::Cmd(Cmd::SW_RESET),
    // observes the BUSY line
    InitStep::WaitUntilIdle,
    // Ram data entry mode = 0x03
    InitStep::CmdData(Cmd::DATA_ENTRY_MODE, &[0x03]),
    // Border / waveform: single-byte value 0x05 in the original
    InitStep::CmdData(Cmd::BORDER_WAVEFORM_CTRL, &[0x05]),
    // VCOM voltage
    InitStep::CmdData(Cmd::WRITE_VCOM_REG, &[0x36]),
    // Gate voltage
    InitStep::CmdData(Cmd::GATE_VOLTAGE_CTRL, &[0x17]),
    // Source voltage (3 bytes)
    InitStep::CmdData(Cmd::SOURCE_VOLTAGE_CTRL, &[0x41, 0x00, 0x32]),
    // Set RAM counters as in the C code
    InitStep::CmdData(Cmd::SET_RAMX_COUNTER, &[0x01]),
    InitStep::CmdData(Cmd::SET_RAMY_COUNTER, &[0x00, 0x00]),
];

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

/// A configured display with a hardware interface.
pub struct Ssd1680<SPI, BSY, DC, RST> {
    interface: SpiDisplayInterface<SPI, BSY, DC, RST>,
    width: u16,
    height: u16,
    /// Panel-specific init sequence provided at construction time.
    init_sequence: &'static [InitStep],
}

impl<SPI, BSY, DC, RST> Ssd1680<SPI, BSY, DC, RST>
where
    SPI: SpiDevice,
    BSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
{
    /// Create the display driver from a concrete SPI device and pins.
    pub fn from_spi(
        spi: SPI,
        busy: BSY,
        dc: DC,
        rst: RST,
        width: u16,
        height: u16,
    ) -> Result<Self, DisplayError> {
        debug!("creating new Ssd1680 instance (SPI)");
        let interface = SpiDisplayInterface::new(spi, busy, dc, rst);
        Ok(Ssd1680 {
            interface,
            width,
            height,
            init_sequence: DEFAULT_INIT_SEQUENCE,
        })
    }

    /// Create with a custom init sequence when using the SPI convenience
    /// constructor.
    pub fn from_spi_with_init_sequence(
        spi: SPI,
        busy: BSY,
        dc: DC,
        rst: RST,
        width: u16,
        height: u16,
        init_sequence: &'static [InitStep],
    ) -> Result<Self, DisplayError> {
        debug!("creating new Ssd1680 instance");
        let interface = SpiDisplayInterface::new(spi, busy, dc, rst);
        Ok(Ssd1680 {
            interface,
            width,
            height,
            init_sequence,
        })
    }

    /// reset the display and put it into a low-power state
    pub fn begin(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        self.interface.hard_reset(delay);
        self.power_down(delay)
    }

    /// Power up the controller
    pub fn power_up(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        debug!("powering up ssd1680");
        // hard reset
        self.interface.hard_reset(delay);
        for step in self.init_sequence {
            debug!("init step: {:?}", step);
            match *step {
                InitStep::SWReset => {
                    debug!("InitStep::Reset");
                    self.interface.cmd(cmd::Cmd::SW_RESET)?;
                }
                InitStep::DelayMs(ms) => {
                    debug!("InitStep::DelayMs({})", ms);
                    delay.delay_ms(u32::from(ms));
                }
                InitStep::WaitUntilIdle => {
                    debug!("InitStep::WaitUntilIdle");
                    self.interface.wait_until_idle(delay);
                }
                InitStep::Cmd(c) => {
                    debug!("InitStep::Cmd");
                    self.interface.cmd(c)?;
                }
                InitStep::CmdData(c, d) => {
                    debug!("InitStep::CmdData");
                    self.interface.cmd_with_data(c, d)?;
                }
                InitStep::DriverControl => {
                    // send height-1 as two bytes (low, high) followed by 0x00.
                    // Some panels use a 16-bit height field; sending only a low
                    // byte truncates heights > 255 and corrupts the display.
                    let h = self.height.saturating_sub(1);
                    let low = (h & 0xFF) as u8;
                    // DRIVER_OUTPUT_CTRL uses a 9-bit height field. Only the
                    // LSB of the high byte (bit 8) is meaningful. Mask to 1 bit
                    // to avoid setting unrelated bits in that byte.
                    let high = ((h >> 8) & 0x01) as u8;
                    debug!("InitStep::DriverControl - height={}", h);
                    self.interface
                        .cmd_with_data(Cmd::DRIVER_OUTPUT_CTRL, &[low, high, 0x00])?;
                }
                InitStep::UseFullFrame => {
                    debug!("InitStep::UseFullFrame");
                    self.use_full_frame()?;
                }
            }
        }
        // after the sequence, ensure device is idle
        debug!("init sequence complete");
        self.interface.wait_until_idle(delay);
        Ok(())
    }

    fn power_down(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        debug!("powering down ssd1680");
        // deep sleep
        self.interface.cmd_with_data(Cmd::DEEP_SLEEP, &[0x01])?;
        delay.delay_ms(1);
        Ok(())
    }

    /// Start an update of the whole display
    pub fn display(
        &mut self,
        bw_buffer: &[u8],
        red_buffer: &[u8],
        delay: &mut impl DelayNs,
    ) -> Result<(), DisplayError> {
        debug!("displaying buffer on ssd1680");
        self.power_up(delay)?;

        self.write_bw_ram(bw_buffer)?;
        self.write_red_ram(red_buffer)?;

        self.interface.wait_until_idle(delay);

        self.update_display(delay)?;

        self.interface.wait_until_idle(delay);

        self.power_down(delay)?;

        Ok(())
    }

    /// Start an update of the whole display
    pub fn display_bw(
        &mut self,
        buffer: &[u8],
        delay: &mut impl DelayNs,
    ) -> Result<(), DisplayError> {
        debug!("displaying buffer on ssd1680");
        self.power_up(delay)?;

        self.write_bw_ram(buffer)?;

        self.interface.wait_until_idle(delay);

        self.update_display(delay)?;

        self.power_down(delay)?;

        Ok(())
    }

    /// Start an update of the whole display
    pub fn display_red(
        &mut self,
        buffer: &[u8],
        delay: &mut impl DelayNs,
    ) -> Result<(), DisplayError> {
        debug!("displaying buffer on ssd1680");
        self.power_up(delay)?;

        self.write_red_ram(buffer)?;

        self.interface.wait_until_idle(delay);

        self.update_display(delay)?;

        self.power_down(delay)?;

        Ok(())
    }

    fn update_display(&mut self, delay: &mut impl DelayNs) -> Result<(), DisplayError> {
        debug!("updating display on ssd1680");
        self.interface
            .cmd_with_data(Cmd::DISPLAY_UPDATE_CTRL2, &[Flag::DISPLAY_MODE_LOAD_TEMP_1])?;
        self.interface.cmd(Cmd::MASTER_ACTIVATE)?;

        self.interface.wait_until_idle(delay);
        Ok(())
    }

    /// Update the whole BW buffer on the display driver
    fn write_bw_ram(&mut self, buffer: &[u8]) -> Result<(), DisplayError> {
        debug!("writing BW RAM to ssd1680");
        // start from the beginning
        self.set_ram_counter(0, 0)?;
        self.interface.cmd_with_data(Cmd::WRITE_BW_DATA, buffer)
    }

    /// Update the whole Red buffer on the display driver
    fn write_red_ram(&mut self, buffer: &[u8]) -> Result<(), DisplayError> {
        debug!("writing Red RAM to ssd1680");
        // start from the beginning
        self.set_ram_counter(0, 0)?;
        self.interface.cmd_with_data(Cmd::WRITE_RED_DATA, buffer)
    }

    /// Make the whole black and white RAM on the display driver white
    pub fn clear_bw_ram(&mut self) -> Result<(), DisplayError> {
        debug!("clearing BW RAM on ssd1680");

        // TODO: allow non-white background color
        let color = color::Color::White.get_byte_value();

        self.interface.cmd(Cmd::WRITE_BW_DATA)?;
        // use ceil division for bytes-per-row so widths not divisible by 8
        // are handled correctly (matches buffer_len / find_position logic).
        self.interface.data_x_times(
            color,
            u32::from(self.width).div_ceil(8) * u32::from(self.height),
        )?;
        Ok(())
    }

    /// Make the whole red RAM on the display driver white
    pub fn clear_red_ram(&mut self) -> Result<(), DisplayError> {
        debug!("clearing red RAM on ssd1680");

        // TODO: allow non-white background color
        let color = color::Color::White.inverse().get_byte_value();

        self.interface.cmd(Cmd::WRITE_RED_DATA)?;
        // use ceil division for bytes-per-row so widths not divisible by 8
        // are handled correctly (matches buffer_len / find_position logic).
        self.interface.data_x_times(
            color,
            u32::from(self.width).div_ceil(8) * u32::from(self.height),
        )?;
        Ok(())
    }

    fn use_full_frame(&mut self) -> Result<(), DisplayError> {
        debug!("setting full frame on ssd1680");
        // choose full frame/ram
        self.set_ram_area(0, 0, self.width - 1, self.height - 1)?;

        // start from the beginning
        self.set_ram_counter(0, 0)
    }

    fn set_ram_area(
        &mut self,
        start_x: u16,
        start_y: u16,
        end_x: u16,
        end_y: u16,
    ) -> Result<(), DisplayError> {
        assert!(start_x < end_x);
        assert!(start_y < end_y);

        debug!(
            "set_ram_area: x {}-{}, y {}-{}",
            start_x, end_x, start_y, end_y
        );

        // x is positioned in bytes, so the last 3 bits which show the position inside a byte in the ram
        // aren't relevant
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
        // x is positioned in bytes, so the last 3 bits which show the position inside a byte in the ram
        // aren't relevant
        debug!("set_ram_counter: x {}, y {}", x, y);
        self.interface
            .cmd_with_data(Cmd::SET_RAMX_COUNTER, &[(x >> 3) as u8])?;

        // 2 Databytes: A[7:0] & 0..A[8]
        self.interface.cmd_with_data(
            Cmd::SET_RAMY_COUNTER,
            &[(y & 0xFF) as u8, ((y >> 8) & 0x01) as u8],
        )?;
        Ok(())
    }
}
