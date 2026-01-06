#![no_std]

//! M24512 EEPROM Driver
//!
//! The M24512 is a 512-Kbit I2C-compatible EEPROM (64 KB).
//!
//! Features:
//! - 16-bit memory address
//! - 128-byte page write
//! - Write cycle time: 5ms (max)
//! - Device address: 0x50-0x57 base

use core::fmt::Debug;
use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::i2c::I2c;

use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind};
use embedded_storage_async::nor_flash::{
    MultiwriteNorFlash as AsyncMultiwriteNorFlash, NorFlash as AsyncNorFlash,
    ReadNorFlash as AsyncReadNorFlash,
};

/// M24512 errors
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum M24512Error<E> {
    /// I2C communication error
    I2c(E),
    /// Invalid memory address (must be < 65536)
    InvalidAddress,
    /// Write operation exceeds page boundary
    PageBoundaryExceeded,
    /// Invalid data length
    InvalidLength,
    /// Write operation attempted while write protection is enabled
    WriteProtected,
    /// Timeout waiting for device ready
    Timeout,
}

impl<E: Debug> NorFlashError for M24512Error<E> {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            M24512Error::I2c(_) => NorFlashErrorKind::Other,
            M24512Error::InvalidAddress => NorFlashErrorKind::OutOfBounds,
            M24512Error::PageBoundaryExceeded => NorFlashErrorKind::Other,
            M24512Error::InvalidLength => NorFlashErrorKind::Other,
            M24512Error::WriteProtected => NorFlashErrorKind::Other,
            M24512Error::Timeout => NorFlashErrorKind::Other,
        }
    }
}

/// M24512 driver
pub struct M24512<I2C, WC> {
    /// I2C device address
    device_address: u8,
    /// I2C interface
    i2c: I2C,
    /// Write control pin (nWC) - Active Low
    write_control_pin: WC,
    /// Internal write enabled state
    write_enabled: bool,
}

impl<I2C, WC, E> M24512<I2C, WC>
where
    I2C: I2c<Error = E>,
    WC: OutputPin,
    E: Debug,
{
    /// M24512 total memory size in bytes (64 KB)
    pub const MEMORY_SIZE: usize = 65536;

    /// Page size for page write operations (128 bytes)
    pub const PAGE_SIZE: usize = 128;

    /// Maximum write cycle time in milliseconds
    pub const WRITE_CYCLE_TIME_MS: u64 = 5;

    /// Default I2C device address (A2=A1=A0=0)
    pub const DEFAULT_ADDRESS: u8 = 0x50;

    /// Create a new M24512 driver instance
    ///
    /// # Parameters
    /// - `i2c`: I2C interface
    /// - `write_control_pin`: Output pin for write control (nWC)
    /// - `device_address`: I2C device address (typically 0x50)
    pub fn new(i2c: I2C, mut write_control_pin: WC, device_address: u8) -> Self {
        // Start with write protection enabled (nWC = HIGH)
        let _ = write_control_pin.set_high();

        Self {
            device_address,
            i2c,
            write_control_pin,
            write_enabled: false,
        }
    }

    /// Create a new M24512 driver with default address (0x50)
    pub fn new_default(i2c: I2C, write_control_pin: WC) -> Self {
        Self::new(i2c, write_control_pin, Self::DEFAULT_ADDRESS)
    }

    /// Enable write operations (nWC = LOW)
    pub fn enable_write(&mut self) -> Result<(), WC::Error> {
        self.write_enabled = true;
        self.write_control_pin.set_low()
    }

    /// Disable write operations (nWC = HIGH)
    pub fn disable_write(&mut self) -> Result<(), WC::Error> {
        self.write_enabled = false;
        self.write_control_pin.set_high()
    }

    /// Check if write is enabled
    pub fn is_write_enabled(&self) -> bool {
        self.write_enabled
    }

    /// Read a single byte
    pub async fn read_byte(&mut self, address: u16) -> Result<u8, M24512Error<E>> {
        let mut buffer = [0u8; 1];
        self.read_bytes(address, &mut buffer).await?;
        Ok(buffer[0])
    }

    /// Read multiple bytes sequentially
    pub async fn read_bytes(&mut self, address: u16, buffer: &mut [u8]) -> Result<(), M24512Error<E>> {
        if address as usize + buffer.len() > Self::MEMORY_SIZE {
            return Err(M24512Error::InvalidAddress);
        }

        let addr_bytes = address.to_be_bytes();
        self.i2c
            .write_read(self.device_address, &addr_bytes, buffer)
            .await
            .map_err(M24512Error::I2c)
    }

    /// Write bytes, handling page boundaries automatically
    pub async fn write_bytes(&mut self, address: u16, data: &[u8]) -> Result<(), M24512Error<E>> {
        if address as usize + data.len() > Self::MEMORY_SIZE {
            return Err(M24512Error::InvalidAddress);
        }

        if data.is_empty() {
            return Ok(());
        }

        let mut current_address = address;
        let mut bytes_written = 0;

        while bytes_written < data.len() {
            let page_offset = (current_address as usize) % Self::PAGE_SIZE;
            let bytes_remaining_in_page = Self::PAGE_SIZE - page_offset;
            let bytes_to_write = core::cmp::min(
                bytes_remaining_in_page,
                data.len() - bytes_written,
            );

            self.write_page(
                current_address,
                &data[bytes_written..bytes_written + bytes_to_write],
            )
            .await?;

            bytes_written += bytes_to_write;
            current_address += bytes_to_write as u16;
        }

        Ok(())
    }

    /// Write data within a single page
    async fn write_page(&mut self, address: u16, data: &[u8]) -> Result<(), M24512Error<E>> {
        if data.is_empty() || data.len() > Self::PAGE_SIZE {
            return Err(M24512Error::InvalidLength);
        }

        let addr_bytes = address.to_be_bytes();

        let mut write_buf = heapless::Vec::<u8, 130>::new();
        write_buf.extend_from_slice(&addr_bytes).ok();
        write_buf.extend_from_slice(data).ok();

        self.i2c
            .write(self.device_address, &write_buf)
            .await
            .map_err(M24512Error::I2c)?;

        // Wait for write cycle
        Timer::after(Duration::from_millis(Self::WRITE_CYCLE_TIME_MS)).await;

        Ok(())
    }

    /// Poll device for readiness
    pub async fn is_ready(&mut self) -> Result<bool, M24512Error<E>> {
        match self.i2c.write(self.device_address, &[]).await {
            Ok(_) => Ok(true),
            Err(_) => Ok(false),
        }
    }

    /// Wait for device to be ready
    pub async fn wait_ready(&mut self, timeout_ms: u64) -> Result<(), M24512Error<E>> {
        let start = embassy_time::Instant::now();
        let timeout = Duration::from_millis(timeout_ms);

        while embassy_time::Instant::now() - start < timeout {
            if self.is_ready().await? {
                return Ok(());
            }
            Timer::after(Duration::from_millis(1)).await;
        }

        Err(M24512Error::Timeout)
    }

    /// Get total capacity
    pub const fn capacity(&self) -> usize {
        Self::MEMORY_SIZE
    }

    /// Get the device I2C address
    pub fn device_address(&self) -> u8 {
        self.device_address
    }

    /// Destroy driver and return components
    pub fn destroy(self) -> (I2C, WC) {
        (self.i2c, self.write_control_pin)
    }
}

impl<I2C, WC, E> ErrorType for M24512<I2C, WC>
where
    I2C: I2c<Error = E>,
    E: Debug,
{
    type Error = M24512Error<E>;
}

impl<I2C, WC, E> AsyncReadNorFlash for M24512<I2C, WC>
where
    I2C: I2c<Error = E>,
    WC: OutputPin,
    E: Debug,
{
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.read_bytes(offset as u16, bytes).await
    }

    fn capacity(&self) -> usize {
        Self::MEMORY_SIZE
    }
}

impl<I2C, WC, E> AsyncNorFlash for M24512<I2C, WC>
where
    I2C: I2c<Error = E>,
    WC: OutputPin,
    E: Debug,
{
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = Self::PAGE_SIZE;

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let _ = self.enable_write();
        let result = self.write_bytes(offset as u16, bytes).await;
        let _ = self.disable_write();
        result
    }

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let _ = self.enable_write();

        let mut offset = from;
        while offset < to {
            let chunk_size = core::cmp::min(128, (to - offset) as usize);
            let erase_data = [0xFFu8; 128];
            self.write_bytes(offset as u16, &erase_data[..chunk_size]).await?;
            offset += chunk_size as u32;
        }

        let _ = self.disable_write();
        Ok(())
    }
}

impl<I2C, WC, E> AsyncMultiwriteNorFlash for M24512<I2C, WC>
where
    I2C: I2c<Error = E>,
    WC: OutputPin,
    E: Debug,
{}
