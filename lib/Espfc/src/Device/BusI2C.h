#ifndef _ESPFC_DEVICE_BUSI2C_H_
#define _ESPFC_DEVICE_BUSI2C_H_

#include "BusDevice.h"

#if defined(ESPFC_I2C_0_SOFT)
#include "EspWire.h"
#define WireClass EspTwoWire
#define WireImpl EspWire
#else
#include "Wire.h"
#define WireClass TwoWire
#define WireImpl Wire
#endif

namespace Espfc {

namespace Device {

class BusI2C: public BusDevice
{
  public:
    BusType getType() const override { return BUS_I2C; }

    int begin(int sda, int scl, int speed)
    {
      #if defined(ESPFC_I2C_0_SOFT)
        WireImpl.begin(sda, scl);
      #else
        if(!WireImpl.setSCL(scl)) return -1;
        if(!WireImpl.setSDA(sda)) return -2;
      #endif
      WireImpl.setClock(speed);
      WireImpl.begin();
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

      WireImpl.beginTransmission(devAddr);
      WireImpl.write(regAddr);
      WireImpl.endTransmission();
      WireImpl.requestFrom(devAddr, length);

      for (; WireImpl.available() && (timeout == 0 || millis() - t1 < timeout); count++)
      {
        data[count] = WireImpl.read();
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

      WireImpl.beginTransmission(devAddr);
      WireImpl.write((uint8_t) regAddr); // send address
      for (uint8_t i = 0; i < length; i++)
      {
        WireImpl.write(data[i]);
        //Serial.print("I2C W "); Serial.print(i); Serial.print(' '); Serial.println(data[i], HEX);
      }
      uint8_t status = WireImpl.endTransmission();

      if(onError && status != 0) onError();

      return status == 0;
    }

  private:
#if defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_TWOWIRE)
    WireClass WireImpl;
#endif

};

}

}

#endif
