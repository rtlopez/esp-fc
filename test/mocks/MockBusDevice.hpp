#pragma once

#include "Hal/BusDevice.hpp"
#include <cstdint>

class MockBusDevice : public Hal::BusDevice
{
public:
  uint8_t readRegs[256] = {};
  uint8_t writeRegs[256] = {};
  int writeCalls = 0;
  bool failRead = false;
  Hal::BusType type = Hal::BUS_I2C;

  Hal::BusType getType() const override
  {
    return type;
  }

  int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override
  {
    if (failRead) return 0;
    for (uint8_t i = 0; i < length; i++)
      data[i] = readRegs[(regAddr + i) & 0xFF];
    return length;
  }

  int8_t readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override
  {
    return read(devAddr, regAddr, length, data);
  }

  bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data) override
  {
    for (uint8_t i = 0; i < length; i++)
      writeRegs[(regAddr + i) & 0xFF] = data[i];
    writeCalls++;
    return true;
  }
};
