#pragma once

#include "WirelessDevice.hpp"
#include "Device/BusDevice.hpp"

namespace Espfc::Device::Wireless {

class NRF24L01 : public WirelessDevice
{
public:
  int begin(BusDevice* bus, uint8_t addr) override;
  WirelessDeviceType getType() const override;
  bool testConnection() override;

  // Wireless operations
  int send(const uint8_t* data, uint8_t length) override;
  int receive(uint8_t* data, uint8_t* length) override;
  bool isDataAvailable() override;
  int setAddress(const uint8_t* addr, uint8_t length) override;
  int setChannel(uint8_t channel) override;
  int setPower(uint8_t power) override;

  // NRF24L01 specific
  int flushTx();
  int flushRx();
  int getRxPayloadSize();
  uint8_t getStatus();

private:
  // SPI Commands
  static constexpr uint8_t CMD_R_REGISTER = 0x00;
  static constexpr uint8_t CMD_W_REGISTER = 0x20;
  static constexpr uint8_t CMD_R_RX_PAYLOAD = 0x61;
  static constexpr uint8_t CMD_W_TX_PAYLOAD = 0xA0;
  static constexpr uint8_t CMD_FLUSH_TX = 0xE1;
  static constexpr uint8_t CMD_FLUSH_RX = 0xE2;
  static constexpr uint8_t CMD_REUSE_TX_PL = 0xE3;
  static constexpr uint8_t CMD_R_RX_PL_WID = 0x60;
  static constexpr uint8_t CMD_W_ACK_PAYLOAD = 0xA8;
  static constexpr uint8_t CMD_NOP = 0xFF;

  // Register addresses
  static constexpr uint8_t REG_CONFIG = 0x00;
  static constexpr uint8_t REG_EN_AA = 0x01;
  static constexpr uint8_t REG_EN_RXADDR = 0x02;
  static constexpr uint8_t REG_SETUP_AW = 0x03;
  static constexpr uint8_t REG_SETUP_RETR = 0x04;
  static constexpr uint8_t REG_RF_CH = 0x05;
  static constexpr uint8_t REG_RF_SETUP = 0x06;
  static constexpr uint8_t REG_STATUS = 0x07;
  static constexpr uint8_t REG_OBSERVE_TX = 0x08;
  static constexpr uint8_t REG_RPD = 0x09;
  static constexpr uint8_t REG_RX_ADDR_P0 = 0x0A;
  static constexpr uint8_t REG_RX_ADDR_P1 = 0x0B;
  static constexpr uint8_t REG_RX_ADDR_P2 = 0x0C;
  static constexpr uint8_t REG_RX_ADDR_P3 = 0x0D;
  static constexpr uint8_t REG_RX_ADDR_P4 = 0x0E;
  static constexpr uint8_t REG_RX_ADDR_P5 = 0x0F;
  static constexpr uint8_t REG_TX_ADDR = 0x10;
  static constexpr uint8_t REG_RX_PW_P0 = 0x11;
  static constexpr uint8_t REG_RX_PW_P1 = 0x12;
  static constexpr uint8_t REG_RX_PW_P2 = 0x13;
  static constexpr uint8_t REG_RX_PW_P3 = 0x14;
  static constexpr uint8_t REG_RX_PW_P4 = 0x15;
  static constexpr uint8_t REG_RX_PW_P5 = 0x16;
  static constexpr uint8_t REG_FIFO_STATUS = 0x17;
  static constexpr uint8_t REG_DYNPD = 0x1C;
  static constexpr uint8_t REG_FEATURE = 0x1D;

  // Status bits
  static constexpr uint8_t STATUS_RX_DR = 0x40;
  static constexpr uint8_t STATUS_TX_DS = 0x20;
  static constexpr uint8_t STATUS_MAX_RT = 0x10;
  static constexpr uint8_t STATUS_RX_P_NO = 0x0E;
  static constexpr uint8_t STATUS_TX_FULL = 0x01;

  // FIFO status bits
  static constexpr uint8_t FIFO_RX_EMPTY = 0x01;
  static constexpr uint8_t FIFO_RX_FULL = 0x02;
  static constexpr uint8_t FIFO_TX_EMPTY = 0x10;
  static constexpr uint8_t FIFO_TX_FULL = 0x20;

  uint8_t readReg(uint8_t reg);
  void writeReg(uint8_t reg, uint8_t data);
  uint8_t readRegByte(uint8_t reg, uint8_t* data, uint8_t length);
  uint8_t writeRegByte(uint8_t reg, const uint8_t* data, uint8_t length);
  uint8_t sendCmd(uint8_t cmd);
};

} // namespace Espfc::Device::Wireless
