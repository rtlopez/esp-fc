#pragma once

#include "Hal/BusDevice.hpp"
#include <cstddef>

namespace Espfc::Hal {

class BusSPI : public BusDevice
{
public:
  BusSPI(size_t index);

  static constexpr uint8_t SPI_READ = 0x80;
  static constexpr uint8_t SPI_WRITE = 0x7f;

  static constexpr uint32_t SPI_SPEED_NORMAL = 1000000;
  static constexpr uint32_t SPI_SPEED_FAST = 16000000;

  BusType getType() const override;

  int begin(int8_t sck = -1, int8_t mosi = -1, int8_t miso = -1, int8_t ss = -1);

  int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override;

  int8_t readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override;

  bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data) override;

private:
  void transfer(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* in, uint8_t* out, uint32_t speed);

  size_t _index;
};

BusSPI* getBusSPI(size_t index);

} // namespace Espfc::Hal
