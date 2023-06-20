#ifndef _ESPFC_DEVICE_BUSI2C_H_
#define _ESPFC_DEVICE_BUSI2C_H_

#include "BusDevice.h"

namespace Espfc {

namespace Device {

class BusI2C: public BusDevice
{
  public:
    BusI2C(WireClass& i2c): _dev(i2c) {}

    BusType getType() const override { return BUS_I2C; }

    int begin(int sda, int scl, int speed)
    {
      if(sda == -1 || scl == -1) return 0;

      targetI2CInit(_dev, sda, scl, speed);
      return 1;
    }

    int8_t readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout = ESPFC_BUS_TIMEOUT) override
    {
      return read(devAddr, regAddr, length, data, timeout);
    }

    int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout = ESPFC_BUS_TIMEOUT) override
    {
      int8_t count = 0;
      uint32_t t1 = millis();

      //D("i2c:r0", devAddr, regAddr, length);

      _dev.beginTransmission(devAddr);
      _dev.write(regAddr);
      _dev.endTransmission();
      _dev.requestFrom(devAddr, length);

      for (; _dev.available() && (timeout == 0 || millis() - t1 < timeout); count++)
      {
        data[count] = _dev.read();
        //D("i2c:r1", count, data[count]);
      }

      //D("i2c:r3", length, count);
      if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; // timeout

      if(onError && count != length) onError();

      return count;
    }

    bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override
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

  private:
    WireClass& _dev;
};

}

}

#endif
