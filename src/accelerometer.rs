#![allow(unused)]

use embedded_hal_02::blocking::delay::DelayMs;
use embedded_hal_02::blocking::spi::Transfer;
use embedded_hal_02::digital::v2::OutputPin;
use lis3dsh::interface::Spi as Lis3dshSpi;
use lis3dsh::Lis3dsh;

pub struct Accelerometer<SPI, CS>
where
    SPI: Transfer<u8>,
    CS: OutputPin,
{
    driver: Lis3dsh<Lis3dshSpi<SPI, CS>>,
}

impl<SPI, CS> Accelerometer<SPI, CS>
where
    SPI: Transfer<u8>,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs: CS, delay: &mut impl DelayMs<u8>) -> Result<Self, SPI::Error> {
        let mut driver = Lis3dsh::new_spi(spi, cs);
        driver.init(delay)?;
        Ok(Self { driver })
    }

    pub fn read_raw(&mut self) -> Result<[i16; 3], SPI::Error> {
        self.driver.read_data()
    }

    pub fn read_xy_g(&mut self) -> Result<(f32, f32), SPI::Error> {
        let [x, y, _z] = self.read_raw()?;

        // Convert raw readings to g's (assuming ±2g range).
        // This is a common fixed-point convention for 16-bit ±2g sensors.
        const LSB_PER_G: f32 = 16_384.0;

        Ok((x as f32 / LSB_PER_G, y as f32 / LSB_PER_G))
    }

    pub fn into_inner(self) -> (SPI, CS) {
        self.driver.into_inner()
    }
}