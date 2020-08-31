#ifndef _ESPFC_DEVICE_BUSI2C_H_
#define _ESPFC_DEVICE_BUSI2C_H_

#include "BusDevice.h"

#define USE_ESP_WIRE

#if defined(USE_ESP_WIRE)
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
      WireImpl.begin(sda, scl);
      WireImpl.setClock(speed);
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

      //Serial.print("I2C R "); Serial.print(devAddr, HEX); Serial.print(' '); Serial.print(regAddr, HEX); Serial.print(' '); Serial.println(length);

      WireImpl.beginTransmission(devAddr);
      WireImpl.write(regAddr);
      WireImpl.endTransmission();
      WireImpl.beginTransmission(devAddr);
      WireImpl.requestFrom(devAddr, length);

      for (; WireImpl.available() && (timeout == 0 || millis() - t1 < timeout); count++)
      {
        data[count] = WireImpl.read();
        //Serial.print("I2C R "); Serial.print(count); Serial.print(' '); Serial.println(data[count], HEX);
      }

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
