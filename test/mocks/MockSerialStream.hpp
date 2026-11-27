#pragma once

#include "Stream/ReadWritable.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

class MockSerialStream : public Espfc::Stream::ReadWritable
{
public:
  std::vector<uint8_t> data;
  std::deque<uint8_t> rx;
  int space = 1024;

  void begin(const Hal::SerialDeviceConfig&) override {}
  void updateBaudRate(int) override {}
  int available() override
  {
    return rx.size();
  }
  int read() override
  {
    if (rx.empty()) return -1;
    const uint8_t c = rx.front();
    rx.pop_front();
    return c;
  }
  size_t readMany(uint8_t* d, size_t len) override
  {
    const size_t n = std::min(len, rx.size());
    for (size_t i = 0; i < n; i++)
    {
      d[i] = rx.front();
      rx.pop_front();
    }
    return n;
  }
  int peek() override
  {
    return rx.empty() ? -1 : rx.front();
  }
  size_t write(uint8_t c) override
  {
    data.push_back(c);
    return 1;
  }
  size_t write(const uint8_t* d, size_t len) override
  {
    data.insert(data.end(), d, d + len);
    return len;
  }
  int availableForWrite() override
  {
    return space;
  }
  void flush() override {}
  bool isTxFifoEmpty() override
  {
    return true;
  }
};
