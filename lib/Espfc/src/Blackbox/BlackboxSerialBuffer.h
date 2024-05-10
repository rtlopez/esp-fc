#pragma once

#include "Target/Target.h"
#include "Device/SerialDevice.h"

namespace Espfc {

namespace Blackbox {

class BlackboxSerialBuffer: public Device::SerialDevice
{
  public:
    BlackboxSerialBuffer(): _dev(nullptr), _idx(0) {}

    virtual void wrap(Espfc::Device::SerialDevice * s)
    {
      _dev = s;
    }

    virtual void begin(const Espfc::SerialDeviceConfig& conf)
    {
      //_dev->begin(conf);
    }

    virtual size_t write(uint8_t c)
    {
      _data[_idx++] = c;
      if(_idx >= SIZE) flush();
      return 1;
    }

    virtual void flush()
    {
      if(_dev) _dev->write(_data, _idx);
      _idx = 0;
    }

    virtual int availableForWrite()
    {
      //return _dev->availableForWrite();
      return SIZE - _idx;
    }

    virtual bool isTxFifoEmpty()
    {
      //return _dev->isTxFifoEmpty();
      return _idx == 0;
    }

    virtual int available() { return _dev->available(); }
    virtual int read() { return _dev->read(); }
    virtual size_t readMany(uint8_t * c, size_t l) {
#ifdef TARGET_RP2040
      size_t count = std::min(l, (size_t)available());
      for(size_t i = 0; i < count; i++)
      {
        c[i] = read();
      }
      return count;
#else
      return _dev->readMany(c, l);
#endif
    }
    virtual int peek() { return _dev->peek(); }

    virtual size_t write(const uint8_t * c, size_t l)
    {
      for(size_t i = 0; i < l; i++)
      {
        write(c[i]);
      }
      return l;
    }
    virtual bool isSoft() const { return false; };
    virtual operator bool() const { return (bool)(*_dev); }

    static const size_t SIZE = SERIAL_TX_FIFO_SIZE;//128;

    Device::SerialDevice * _dev;
    size_t _idx;
    uint8_t _data[SIZE];
};

}

}
