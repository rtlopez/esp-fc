#if defined(UNIT_TEST)

#include "Hal/BusI2C.hpp"

namespace {

Espfc::Hal::BusI2C busI2C0(0);
Espfc::Hal::BusI2C busI2C1(1);

}

namespace Espfc::Hal {

BusI2C* getBusI2C(size_t index)
{
  switch (index) {
    case 0:
      return &busI2C0;
    case 1:
      return &busI2C1;
    default:
      return nullptr;
  }
}

BusI2C::BusI2C(size_t index): _index(index) {}

BusType BusI2C::getType() const
{
  return BUS_I2C;
}

int BusI2C::begin(int sda, int scl, uint32_t speed)
{
  // do nothing
  return 0;
}

int8_t BusI2C::readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
  // do nothing
  return 0;
}

int8_t BusI2C::read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
  // do nothing
  return 0;
}

bool BusI2C::write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data)
{
  // do nothing
  return false;
}

} // namespace Espfc::Hal

#endif
