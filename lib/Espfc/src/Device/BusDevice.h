#ifndef _ESPFC_DEVICE_BUSDEVICE_H_
#define _ESPFC_DEVICE_BUSDEVICE_H_

#include <functional>
#include <cstdint>
#include "Math/Bits.h"

#define ESPFC_BUS_TIMEOUT 100

namespace Espfc {

enum BusType {
  BUS_NONE,
  BUS_AUTO,
  BUS_I2C,
  BUS_SPI,
  BUS_SLV,
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

    int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
    {
      uint8_t b;
      uint8_t count = readByte(devAddr, regAddr, &b);
      *data = Math::getBit(b, bitNum);
      return count;
    }

    int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
    {
      uint8_t count, b;
      if ((count = readByte(devAddr, regAddr, &b)) != 0)
      {
        *data = Math::getBitsMsb(b, bitStart, length);
      }
      return count;
    }

    int8_t readBitsLsb(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
    {
      uint8_t count, b;
      if ((count = readByte(devAddr, regAddr, &b)) != 0)
      {
        *data = Math::getBitsLsb(b, bitStart, length);
      }
      return count;
    }

    bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
    {
      uint8_t b;
      readByte(devAddr, regAddr, &b);
      b = Math::setBit(b, bitNum, data);
      return writeByte(devAddr, regAddr, b);
    }

    bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
    {
      uint8_t b = 0;
      if (readByte(devAddr, regAddr, &b) != 0)
      {
        b = Math::setBitsMsb(b, bitStart, length, data);
        return writeByte(devAddr, regAddr, b);
      } else {
        return false;
      }
    }

    bool writeBitsLsb(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
    {
      uint8_t b = 0;
      if (readByte(devAddr, regAddr, &b) != 0)
      {
        b = Math::setBitsLsb(b, bitStart, length, data);
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
        b = Math::setMasked(b, mask, data);
        return writeByte(devAddr, regAddr, b);
      } else {
        return false;
      }
    }

    static const char ** getNames()
    {
      static const char* busDevChoices[] = { PSTR("NONE"), PSTR("AUTO"), PSTR("I2C"), PSTR("SPI"), PSTR("SLV"), NULL };
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
