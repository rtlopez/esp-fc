#if defined(ARCH_RP2040)

#include "Hal/BusI2C.hpp"
#include <Arduino.h>
#include <Wire.h>

namespace {

TwoWire& getI2C(size_t index)
{
  switch (index)
  {
    case 0:
      return Wire;
    case 1:
      return Wire1;
    default:
      return Wire;
  }
}

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
  if (sda == -1 || scl == -1) return 0;

  auto& dev = getI2C(_index);
  if (!dev.setSCL(scl)) return 0;
  if (!dev.setSDA(sda)) return 0;
  dev.setClock(speed);
  dev.begin();
  dev.setTimeout(50);

  return 1;
}

int8_t BusI2C::readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
  return read(devAddr, regAddr, length, data);
}

int8_t BusI2C::read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
  int8_t count = 0;
  uint32_t t1 = millis();

  auto& dev = getI2C(_index);

  dev.beginTransmission(devAddr);
  dev.write(regAddr);
  dev.endTransmission();
  dev.requestFrom(devAddr, length);

  for (; dev.available() && (_timeout == 0 || millis() - t1 < _timeout); count++)
  {
    data[count] = dev.read();
  }

  if (_timeout > 0 && millis() - t1 >= _timeout && count < length) count = -1; // timeout

  if (onError && count != length) onError();

  return count;
}

bool BusI2C::write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data)
{
  auto& dev = getI2C(_index);

  dev.beginTransmission(devAddr);
  dev.write((uint8_t)regAddr); // send address
  for (uint8_t i = 0; i < length; i++)
  {
    dev.write(data[i]);
  }
  uint8_t status = dev.endTransmission();

  if (onError && status != 0) onError();

  return status == 0;
}

} // namespace Espfc::Hal

#endif
