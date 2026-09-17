#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "Hal/Serial.hpp"
#include "Stream/ReadWritable.hpp"

namespace Espfc::Blackbox {

class BlackboxSerialBuffer : public Stream::ReadWritable
{
public:
  static constexpr size_t SIZE = Hal::SERIAL_TX_BUFFER_SIZE;

  BlackboxSerialBuffer(): _dev(nullptr), _idx(0) {}

  void wrap(Stream::ReadWritable* s)
  {
    _dev = s;
    _data.reset(new uint8_t[SIZE]);
  }

  void begin(const Hal::SerialDeviceConfig& conf) override
  {
    //_dev->begin(conf);
  }

  void updateBaudRate(int baud) override
  {
    //_dev->updateBaudRate(baud);
  }

  size_t write(uint8_t c) override
  {
    _data[_idx++] = c;
    if (_idx >= SIZE) flush();
    return 1;
  }

  void flush() override
  {
    if (_dev) _dev->write(_data.get(), _idx);
    _idx = 0;
  }

  int availableForWrite() override
  {
    // return _dev->availableForWrite();
    return SIZE - _idx;
  }

  bool isTxFifoEmpty() override
  {
    // return _dev->isTxFifoEmpty();
    return _idx == 0;
  }

  int available() override
  {
    return _dev->available();
  }

  int read() override
  {
    return _dev->read();
  }

  size_t readMany(uint8_t* c, size_t l) override
  {
    return _dev->readMany(c, l);
  }

  int peek() override
  {
    return _dev->peek();
  }

  size_t write(const uint8_t* c, size_t l) override
  {
    for (size_t i = 0; i < l; i++)
    {
      write(c[i]);
    }
    return l;
  }

  Stream::ReadWritable* _dev;
  size_t _idx;
  std::unique_ptr<uint8_t[]> _data;
};

} // namespace Espfc::Blackbox
