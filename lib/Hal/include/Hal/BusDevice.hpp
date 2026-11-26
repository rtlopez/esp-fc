#pragma once

#include <cstdint>
#include <functional>

constexpr uint32_t ESPFC_BUS_TIMEOUT = 100u;

namespace Hal {

enum BusType
{
  BUS_NONE,
  BUS_AUTO,
  BUS_I2C,
  BUS_SPI,
  BUS_SLV,
  BUS_MAX
};

class BusDevice
{
public:
  BusDevice(): _timeout(ESPFC_BUS_TIMEOUT) {}

  virtual BusType getType() const = 0;

  virtual int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) = 0;

  virtual int8_t readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) = 0;

  virtual bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data) = 0;

  bool isSPI() const
  {
    return getType() == BUS_SPI;
  }

  void setTimeout(uint32_t t)
  {
    _timeout = t;
  }

  int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t* data)
  {
    return read(devAddr, regAddr, 1, data);
  }

  bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
  {
    return write(devAddr, regAddr, 1, &data);
  }

  static const char** getNames();
  static const char* getName(BusType type);

  std::function<void(void)> onError;

protected:
  uint32_t _timeout;
};

} // namespace Hal
