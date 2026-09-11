#if defined(UNIT_TEST)

#include "Hal/BusSPI.hpp"

namespace {

Espfc::Hal::BusSPI _spi0(0);
Espfc::Hal::BusSPI _spi1(1);

} // namespace

namespace Espfc::Hal {

BusSPI* getBusSPI(size_t index)
{
  switch (index)
  {
    case 0:
      return &_spi0;
    case 1:
      return &_spi1;
    default:
      return nullptr;
  }
}

BusSPI::BusSPI(size_t index): _index(index) {}

BusType BusSPI::getType() const
{
  return BUS_SPI;
}

int BusSPI::begin(int8_t sck, int8_t mosi, int8_t miso, int8_t ss)
{
  // do nothing
  return 1;
}

int8_t BusSPI::read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
  // do nothing
  return 0;
}

int8_t BusSPI::readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
  // do nothing
  return 0;
}

bool BusSPI::write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data)
{
  // do nothing
  return false;
}

void BusSPI::transfer(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* in, uint8_t* out, uint32_t speed)
{
  // do nothing
}

} // namespace Espfc::Hal

#endif
