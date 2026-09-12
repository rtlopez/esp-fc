#include "NRF24L01.hpp"
#include "Hal/Time.hpp"

namespace Espfc::Device::Wireless {

int NRF24L01::begin(BusDevice* bus, uint8_t addr)
{
  setBus(bus, addr);

  if (!testConnection()) return 0;

  // Power on and configure
  writeReg(REG_CONFIG, 0x0E); // PWR_UP=1, CRCO=1, EN_CRC=1, RX mode
  delay(5); // Wait for power-on

  // Enable CRC (1 byte), 2Mbps data rate, 0dBm TX power
  writeReg(REG_RF_SETUP, 0x0E); // RF_PWR=00 (0dBm), RF_DR_LOW=0, RF_DR_HIGH=1 (2Mbps)

  // Set channel 76 (2.476 GHz)
  writeReg(REG_RF_CH, 76);

  // Set address width to 5 bytes
  writeReg(REG_SETUP_AW, 0x03);

  // Disable auto-acknowledge
  writeReg(REG_EN_AA, 0x00);

  // Enable RX address P0 and P1
  writeReg(REG_EN_RXADDR, 0x03);

  // Set RX payload width for P0 and P1 to 32 bytes
  writeReg(REG_RX_PW_P0, 32);
  writeReg(REG_RX_PW_P1, 32);

  // Set default RX address for P0: 0xC2C2C2C2C2
  uint8_t rxAddr[] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
  writeRegByte(REG_RX_ADDR_P0, rxAddr, 5);

  // Set default RX address for P1: 0xC2C2C2C2C3
  uint8_t rxAddr1[] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC3};
  writeRegByte(REG_RX_ADDR_P1, rxAddr1, 5);

  // Set TX address: 0xC2C2C2C2C2
  writeRegByte(REG_TX_ADDR, rxAddr, 5);

  // Clear all status flags
  writeReg(REG_STATUS, 0x70);

  // Flush RX and TX FIFOs
  flushRx();
  flushTx();

  // Enable RX mode
  writeReg(REG_CONFIG, 0x0F); // PWR_UP=1, CRCO=1, EN_CRC=1, RX mode, EN_RX_DR=1

  return 1;
}

WirelessDeviceType NRF24L01::getType() const
{
  return WIRELESS_NRF24L01;
}

bool NRF24L01::testConnection()
{
  if (!_bus) return false;

  uint8_t setupAW = readReg(REG_SETUP_AW);
  // SETUP_AW should be readable and valid (0x00-0x03)
  return (setupAW <= 0x03);
}

int NRF24L01::send(const uint8_t* data, uint8_t length)
{
  if (!_bus || !data || length == 0 || length > 32) return 0;

  // Limit length to 32 bytes
  if (length > 32) length = 32;

  // Switch to TX mode
  uint8_t config = readReg(REG_CONFIG);
  writeReg(REG_CONFIG, config & ~0x01); // Clear PRIM_RX bit for TX mode

  delay(1);

  // Write TX payload
  writeRegByte(CMD_W_TX_PAYLOAD | 0x00, data, length);

  // Pulse CE to start transmission
  // CE is handled by the caller (via GPIO control)
  
  delay(1);

  // Wait for transmission complete
  for (int i = 0; i < 100; i++)
  {
    uint8_t status = getStatus();
    if (status & STATUS_TX_DS)
    { // TX complete
      writeReg(REG_STATUS, STATUS_TX_DS); // Clear TX complete flag
      return 1;
    }
    if (status & STATUS_MAX_RT)
    { // Max retries reached
      writeReg(REG_STATUS, STATUS_MAX_RT); // Clear flag
      flushTx();
      return 0;
    }
    delay(1);
  }

  return 0;
}

int NRF24L01::receive(uint8_t* data, uint8_t* length)
{
  if (!_bus || !data || !length) return 0;

  if (!isDataAvailable()) return 0;

  uint8_t payloadSize = getRxPayloadSize();
  if (payloadSize == 0 || payloadSize > 32)
  {
    flushRx();
    return 0;
  }

  *length = (payloadSize < *length) ? payloadSize : *length;

  // Read RX payload
  readRegByte(CMD_R_RX_PAYLOAD, data, *length);

  // Clear RX complete flag
  writeReg(REG_STATUS, STATUS_RX_DR);

  return 1;
}

bool NRF24L01::isDataAvailable()
{
  uint8_t status = getStatus();
  return (status & STATUS_RX_DR) != 0;
}

int NRF24L01::setAddress(const uint8_t* addr, uint8_t length)
{
  if (!_bus || !addr || length == 0 || length > 5) return 0;

  // Set address width
  uint8_t addressWidth = length - 2; // 0x01 for 3 bytes, 0x03 for 5 bytes
  if (addressWidth < 1) addressWidth = 1;
  if (addressWidth > 3) addressWidth = 3;
  writeReg(REG_SETUP_AW, addressWidth);

  // Set RX addresses
  writeRegByte(REG_RX_ADDR_P0, addr, length);
  writeRegByte(REG_RX_ADDR_P1, addr, length);

  // Set TX address
  writeRegByte(REG_TX_ADDR, addr, length);

  return 1;
}

int NRF24L01::setChannel(uint8_t channel)
{
  if (!_bus || channel > 125) return 0;

  writeReg(REG_RF_CH, channel);
  return 1;
}

int NRF24L01::setPower(uint8_t power)
{
  if (!_bus) return 0;

  // power: 0 = -18dBm, 1 = -12dBm, 2 = -6dBm, 3 = 0dBm
  if (power > 3) power = 3;

  uint8_t rfSetup = readReg(REG_RF_SETUP);
  rfSetup = (rfSetup & 0xF9) | (power << 1); // Clear and set RF_PWR bits
  writeReg(REG_RF_SETUP, rfSetup);

  return 1;
}

int NRF24L01::flushTx()
{
  if (!_bus) return 0;
  sendCmd(CMD_FLUSH_TX);
  return 1;
}

int NRF24L01::flushRx()
{
  if (!_bus) return 0;
  sendCmd(CMD_FLUSH_RX);
  return 1;
}

int NRF24L01::getRxPayloadSize()
{
  if (!_bus) return 0;

  uint8_t size = 0;
  readRegByte(CMD_R_RX_PL_WID, &size, 1);
  return size;
}

uint8_t NRF24L01::getStatus()
{
  if (!_bus) return 0;
  return sendCmd(CMD_NOP);
}

uint8_t NRF24L01::readReg(uint8_t reg)
{
  uint8_t data = 0;
  readRegByte(reg, &data, 1);
  return data;
}

void NRF24L01::writeReg(uint8_t reg, uint8_t data)
{
  writeRegByte(reg, &data, 1);
}

uint8_t NRF24L01::readRegByte(uint8_t reg, uint8_t* data, uint8_t length)
{
  return _bus->read(_addr, reg, length, data);
}

uint8_t NRF24L01::writeRegByte(uint8_t reg, const uint8_t* data, uint8_t length)
{
  return _bus->write(_addr, reg, length, data) ? length : 0;
}

uint8_t NRF24L01::sendCmd(uint8_t cmd)
{
  return _bus->read(_addr, cmd, 1, nullptr);
}

} // namespace Espfc::Device::Wireless
