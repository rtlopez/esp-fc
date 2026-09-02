#pragma once

#include "Stream/Writable.hpp"

namespace Espfc::Stream {

// Non allocating Writable over a caller supplied buffer, useful for tests and in memory logging.
// One byte of the buffer is reserved for the terminator, so c_str() is always valid.
class BufferWritable : public Writable
{
public:
  BufferWritable(): BufferWritable(nullptr, 0) {}

  BufferWritable(char* buff, size_t size): _buff(buff), _size(size), _len(0), _overflow(false)
  {
    if (_size) _buff[0] = '\0';
  }

  // rebinds the sink to another buffer, the previous one is not touched
  void reset(char* buff, size_t size)
  {
    _buff = buff;
    _size = size;
    _len = 0;
    _overflow = false;
    if (_size) _buff[0] = '\0';
  }

  size_t write(uint8_t c) override
  {
    if (_size < 2 || _len >= _size - 1)
    {
      _overflow = true;
      return 0;
    }
    _buff[_len++] = static_cast<char>(c);
    _buff[_len] = '\0';
    return 1;
  }

  size_t write(const uint8_t* data, size_t len) override
  {
    size_t written = 0;
    for (size_t i = 0; i < len; i++)
    {
      if (!write(data[i])) break;
      written++;
    }
    return written;
  }

  int availableForWrite() override
  {
    return _size < 2 ? 0 : static_cast<int>(_size - 1 - _len);
  }

  void flush() override {}

  bool isTxFifoEmpty() override { return _size == 0; }

  void clear()
  {
    _len = 0;
    _overflow = false;
    if (_size) _buff[0] = '\0';
  }

  const char* c_str() const
  {
    return _buff;
  }

  size_t size() const
  {
    return _len;
  }

  bool overflow() const
  {
    return _overflow;
  }

private:
  char* _buff;
  size_t _size;
  size_t _len;
  bool _overflow;
};

} // namespace Espfc::Stream
