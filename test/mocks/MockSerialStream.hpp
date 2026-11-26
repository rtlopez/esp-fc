#pragma once

#include "Stream/ReadWritable.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

class MockSerialStream : public Espfc::Stream::ReadWritable
{
public:
  std::vector<uint8_t> data;
  int space = 1024;

  void begin(const Hal::SerialDeviceConfig&) override {}
  void updateBaudRate(int) override {}
  int available() override
  {
    return 0;
  }
  int read() override
  {
    return -1;
  }
  size_t readMany(uint8_t*, size_t) override
  {
    return 0;
  }
  int peek() override
  {
    return -1;
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
