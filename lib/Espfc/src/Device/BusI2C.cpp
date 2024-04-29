#include "Target/Target.h"

#if defined(ESPFC_I2C_0)

#include "BusI2C.h"

namespace Espfc {

namespace Device {

BusI2C::BusI2C(WireClass& i2c): _dev(i2c) {}

BusType BusI2C::getType() const { return BUS_I2C; }

int BusI2C::begin(int sda, int scl, uint32_t speed)
{
  if(sda == -1 || scl == -1) return 0;

  targetI2CInit(_dev, sda, scl, speed);

  return 1;
}

int8_t FAST_CODE_ATTR BusI2C::readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
  return read(devAddr, regAddr, length, data);
}

int8_t FAST_CODE_ATTR BusI2C::read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
  int8_t count = 0;
  uint32_t t1 = millis();

  //D("i2c:r0", devAddr, regAddr, length);

  _dev.beginTransmission(devAddr);
  _dev.write(regAddr);
  _dev.endTransmission();
  _dev.requestFrom(devAddr, length);

  for (; _dev.available() && (_timeout == 0 || millis() - t1 < _timeout); count++)
  {
    data[count] = _dev.read();
    //D("i2c:r1", count, data[count]);
  }

  //D("i2c:r3", length, count);
  if (_timeout > 0 && millis() - t1 >= _timeout && count < length) count = -1; // timeout

  if(onError && count != length) onError();

  return count;
}

bool BusI2C::write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data)
{
  //Serial.print("I2C W "); Serial.print(devAddr, HEX); Serial.print(' '); Serial.print(regAddr, HEX); Serial.print(' '); Serial.println(length);

  _dev.beginTransmission(devAddr);
  _dev.write((uint8_t) regAddr); // send address
  for (uint8_t i = 0; i < length; i++)
  {
    _dev.write(data[i]);
    //Serial.print("I2C W "); Serial.print(i); Serial.print(' '); Serial.println(data[i], HEX);
  }
  uint8_t status = _dev.endTransmission();

  if(onError && status != 0) onError();

  return status == 0;
}

}

}

#endif
