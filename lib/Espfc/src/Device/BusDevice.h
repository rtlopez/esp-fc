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
    BusDevice(): _timeout(ESPFC_BUS_TIMEOUT) {}

    virtual BusType getType() const = 0;

    virtual int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) = 0;

    virtual int8_t readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) = 0;

    virtual bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data) = 0;

    bool isSPI() const
    {
      return getType() == BUS_SPI;
    }

    void setTimeout(uint32_t t)
    {
      _timeout = t;
    }

    int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data)
    {
      return read(devAddr, regAddr, 1, data);
    }

    bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
    {
      return write(devAddr, regAddr, 1, &data);
    }

    static bool getBit(uint8_t data, uint8_t bitNum)
    {
      return data & (1 << bitNum);
    }

    static uint8_t setBit(uint8_t data, uint8_t bitNum, uint8_t bitVal)
    {
      data = (bitVal != 0) ? (data | (1 << bitNum)) : (data & ~(1 << bitNum));
      return data;
    }

    static uint8_t setBitsLsb(uint8_t data, uint8_t bitStart, uint8_t bitLen, uint8_t bitVal)
    {
      uint8_t mask = ((1 << bitLen) - 1) << (bitStart - bitLen + 1);
      bitVal <<= (bitStart - bitLen + 1); // shift data into correct position
      bitVal &= mask; // zero all non-important bits in data
      data &= ~(mask); // zero all important bits in existing byte
      data |= bitVal; // combine data with existing byte
      return data;
    }

    static uint8_t setBitsMsb(uint8_t data, uint8_t bitStart, uint8_t bitLen, uint8_t bitVal)
    {
      uint8_t mask = ((1 << bitLen) - 1) << bitStart;
      bitVal <<= bitStart; // shift data into correct position
      bitVal &= mask; // zero all non-important bits in data
      data &= ~(mask); // zero all important bits in existing byte
      data |= bitVal; // combine data with existing byte
      return data;
    }

    static uint8_t getBitsLsb(uint8_t data, uint8_t bitStart, uint8_t bitLen)
    {
      uint8_t mask = ((1 << bitLen) - 1) << (bitStart - bitLen + 1);
      data &= mask;
      data >>= (bitStart - bitLen + 1);
      return data;
    }

    static uint8_t getBitsMsb(uint8_t data, uint8_t bitStart, uint8_t bitLen)
    {
      uint8_t mask = ((1 << bitLen) - 1);
      data >>= bitStart;
      data &= mask;
      return data;
    }

    int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
    {
      uint8_t b;
      uint8_t count = readByte(devAddr, regAddr, &b);
      *data = b & (1 << bitNum);
      return count;
    }

    int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
    {
      uint8_t count, b;
      if ((count = readByte(devAddr, regAddr, &b)) != 0)
      {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
      }
      return count;
    }

    int8_t readBitsBMI160(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
    {
      uint8_t count, b;
      if ((count = readByte(devAddr, regAddr, &b)) != 0)
      {
        uint8_t mask = (1 << length) - 1;
        b >>= bitStart;
        b &= mask;
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

    bool writeBitsBMI160(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
    {
      uint8_t b = 0;
      if (readByte(devAddr, regAddr, &b) != 0)
      {
        uint8_t mask = ((1 << length) - 1) << bitStart;
        data <<= bitStart; // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devAddr, regAddr, b);
      } else {
        return false;
      }
    }

    bool writeMask(uint8_t devAddr, uint8_t regAddr, uint8_t mask, uint8_t data)
    {
      uint8_t b = 0;
      if (readByte(devAddr, regAddr, &b) != 0)
      {
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

  protected:
    uint32_t _timeout;
};

}

}

#endif
