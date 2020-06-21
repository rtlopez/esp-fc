#ifndef _ESPFC_DEVICE_BUSDEVICE_H_
#define _ESPFC_DEVICE_BUSDEVICE_H_

#include <functional>
#include <Arduino.h>

#define ESPFC_BUS_TIMEOUT 100

namespace Espfc {

enum BusType {
  BUS_NONE,
  BUS_AUTO,
  BUS_I2C,
  BUS_SPI,
  BUS_MAX
};

namespace Device {

class BusDevice
{
  public:
    virtual BusType getType() const = 0;

    virtual int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout = ESPFC_BUS_TIMEOUT) = 0;

    virtual int8_t readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout = ESPFC_BUS_TIMEOUT) = 0;

    virtual bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) = 0;

    int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout = ESPFC_BUS_TIMEOUT)
    {
      return read(devAddr, regAddr, 1, data, timeout);
    }

    bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
    {
      return write(devAddr, regAddr, 1, &data);
    }

    int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout = ESPFC_BUS_TIMEOUT)
    {
      uint8_t b;
      uint8_t count = readByte(devAddr, regAddr, &b, timeout);
      *data = b & (1 << bitNum);
      return count;
    }

    int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout = ESPFC_BUS_TIMEOUT)
    {
      uint8_t count, b;
      if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0)
      {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
      }
      return count;
    }

    bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
    {
      uint8_t b;
      readByte(devAddr, regAddr, &b);
      b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
      return writeByte(devAddr, regAddr, b);
    }

    bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
    {
      uint8_t b = 0;
      if (readByte(devAddr, regAddr, &b) != 0)
      {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devAddr, regAddr, b);
      } else {
        return false;
      }
    }

    static const char ** getNames()
    {
      static const char* busDevChoices[] = { PSTR("NONE"), PSTR("AUTO"), PSTR("I2C"), PSTR("SPI"), NULL };
      return busDevChoices;
    }

    static const char * getName(BusType type)
    {
      if(type >= BUS_MAX) return PSTR("?");
      return getNames()[type];
    }

    std::function<void(void)> onError;
};

}

}

#endif
