#ifndef _ESPFC_DEVICE_BUSI2C_H_
#define _ESPFC_DEVICE_BUSI2C_H_

#include "BusDevice.h"
#include "Wire.h"

namespace Espfc {

namespace Device {

class BusI2C: public BusDevice
{
  public:
    int begin(int sda, int scl, int speed)
    {
      Wire.begin(sda, scl);
      Wire.setClock(speed);
      return 1;
    }

    int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout = ESPFC_BUS_TIMEOUT) override
    {
      int8_t count = 0;
      uint32_t t1 = millis();

      Wire.beginTransmission(devAddr);
      Wire.write(regAddr);
      Wire.endTransmission();
      Wire.beginTransmission(devAddr);
      Wire.requestFrom(devAddr, length);

      for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); count++)
      {
        data[count] = Wire.read();
      }

      if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; // timeout

      if(onError && count != length) onError();

      return count;
    }

    bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override
    {
      Wire.beginTransmission(devAddr);
      Wire.write((uint8_t) regAddr); // send address
      for (uint8_t i = 0; i < length; i++)
      {
        Wire.write(data[i]);
      }
      uint8_t status = Wire.endTransmission();

      if(onError && status != 0) onError();

      return status == 0;
    }
};

}

}

#endif
