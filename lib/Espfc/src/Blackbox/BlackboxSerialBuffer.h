#pragma once

#include "Target/Target.h"
#include "Device/SerialDevice.h"

namespace Espfc {

namespace Blackbox {

class BlackboxSerialBuffer: public Device::SerialDevice
{
  public:
    static constexpr size_t SIZE = SERIAL_TX_FIFO_SIZE;//128;

    BlackboxSerialBuffer(): _dev(nullptr), _idx(0), _data(nullptr) {}

    ~BlackboxSerialBuffer()
    {
      if(!_data) return;

      delete[] _data;
      _data = nullptr;
    }

    void updateBaudRate(int baud) override { };

    void wrap(Espfc::Device::SerialDevice * s)
    {
      _dev = s;
      _data = new uint8_t[SIZE];
    }

    void begin(const Espfc::SerialDeviceConfig& conf) override
    {
      //_dev->begin(conf);
    }

    size_t write(uint8_t c) override
    {
      _data[_idx++] = c;
      if(_idx >= SIZE) flush();
      return 1;
    }

    void flush() override
    {
      if(_dev) _dev->write(_data, _idx);
      _idx = 0;
    }

    int availableForWrite() override
    {
      //return _dev->availableForWrite();
      return SIZE - _idx;
    }

    bool isTxFifoEmpty() override
    {
      //return _dev->isTxFifoEmpty();
      return _idx == 0;
    }

    int available() override { return _dev->available(); }
    int read() override { return _dev->read(); }
    size_t readMany(uint8_t * c, size_t l) override {
#if defined(ARCH_RP2040)
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
    int peek() override { return _dev->peek(); }

    size_t write(const uint8_t * c, size_t l) override
    {
      for(size_t i = 0; i < l; i++)
      {
        write(c[i]);
      }
      return l;
    }
    bool isSoft() const override { return false; };
    operator bool() const override { return (bool)(*_dev); }

    Device::SerialDevice * _dev;
    size_t _idx;
    uint8_t* _data;
};

}

}
