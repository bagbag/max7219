use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiDevice;

use crate::DataError;

/// Describes the interface used to connect to the MX7219
pub trait Connector {
    ///
    /// Writes raw bytes
    ///
    /// # Errors
    ///
    /// * `DataError` - returned in case there was an error during data transfer
    ///
    async fn write_raw_bytes(&mut self, bytes: &[u8]) -> Result<(), DataError>;
}

/// Direct GPIO pins connector
pub struct PinConnector<DATA, CS, SCK>
where
    DATA: OutputPin,
    CS: OutputPin,
    SCK: OutputPin,
{
    data: DATA,
    cs: CS,
    sck: SCK,
}

impl<DATA, CS, SCK> PinConnector<DATA, CS, SCK>
where
    DATA: OutputPin,
    CS: OutputPin,
    SCK: OutputPin,
{
    pub(crate) fn new(data: DATA, cs: CS, sck: SCK) -> Self {
        PinConnector { data, cs, sck }
    }
}

impl<DATA, CS, SCK> Connector for PinConnector<DATA, CS, SCK>
where
    DATA: OutputPin,
    CS: OutputPin,
    SCK: OutputPin,
{
    async fn write_raw_bytes(&mut self, bytes: &[u8]) -> Result<(), DataError> {
        self.cs.set_low().map_err(|_| DataError::Pin)?;
        for byte in bytes {
            for i in 0..8 {
                if byte & (1 << (7 - i)) > 0 {
                    self.data.set_high().map_err(|_| DataError::Pin)?;
                } else {
                    self.data.set_low().map_err(|_| DataError::Pin)?;
                }

                self.sck.set_high().map_err(|_| DataError::Pin)?;
                self.sck.set_low().map_err(|_| DataError::Pin)?;
            }
        }
        self.cs.set_high().map_err(|_| DataError::Pin)?;

        Ok(())
    }
}

pub struct SpiConnector<SPI>
where
    SPI: SpiDevice<u8>,
{
    spi: SPI,
}

/// Hardware controlled CS connector with SPI transfer
impl<SPI> SpiConnector<SPI>
where
    SPI: SpiDevice<u8>,
{
    pub(crate) fn new(spi: SPI) -> Self {
        SpiConnector { spi }
    }
}

impl<SPI> Connector for SpiConnector<SPI>
where
    SPI: SpiDevice<u8>,
{
    async fn write_raw_bytes(&mut self, bytes: &[u8]) -> Result<(), DataError> {
        self.spi.write(bytes).await.map_err(|_| DataError::Spi)?;
        Ok(())
    }
}

/// Software controlled CS connector with SPI transfer
pub struct SpiConnectorSW<SPI, CS>
where
    SPI: SpiDevice<u8>,
    CS: OutputPin,
{
    spi_c: SpiConnector<SPI>,
    cs: CS,
}

impl<SPI, CS> SpiConnectorSW<SPI, CS>
where
    SPI: SpiDevice<u8>,
    CS: OutputPin,
{
    pub(crate) fn new(spi: SPI, cs: CS) -> Self {
        SpiConnectorSW {
            spi_c: SpiConnector::new(spi),
            cs,
        }
    }
}

impl<SPI, CS> Connector for SpiConnectorSW<SPI, CS>
where
    SPI: SpiDevice<u8>,
    CS: OutputPin,
{
    async fn write_raw_bytes(&mut self, bytes: &[u8]) -> Result<(), DataError> {
        self.cs.set_low().map_err(|_| DataError::Pin)?;
        self.spi_c
            .write_raw_bytes(bytes)
            .await
            .map_err(|_| DataError::Spi)?;
        self.cs.set_high().map_err(|_| DataError::Pin)?;

        Ok(())
    }
}
