#pragma once

#include "Hal/BusDevice.hpp"
#include <cstddef>

namespace Espfc::Hal {

class BusI2C : public BusDevice
{
public:
  BusI2C(size_t index);
  BusType getType() const override;

  int begin(int sda, int scl, uint32_t speed);
  int8_t readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override;
  int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override;
  bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data) override;

private:
  size_t _index;
};

BusI2C* getBusI2C(size_t index);

} // namespace Espfc::Hal
