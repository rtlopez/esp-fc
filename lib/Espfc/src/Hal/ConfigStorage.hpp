#pragma once

#include <cstddef>
#include <cstdint>

namespace Espfc::Hal {

class ConfigStorage
{
public:
  bool begin(size_t size);
  size_t size() const;

  uint8_t readByte(size_t addr) const;
  void writeByte(size_t addr, uint8_t value);

  size_t read(size_t addr, uint8_t* data, size_t len) const;
  size_t write(size_t addr, const uint8_t* data, size_t len);

  bool commit();

private:
  size_t _size = 0;
};

} // namespace Espfc::Hal
