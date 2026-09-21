#if defined(UNIT_TEST)

#include "Hal/ConfigStorage.hpp"
#include <algorithm>
#include <array>
#include <cstring>

namespace Espfc::Hal {

namespace {

constexpr size_t MAX_SIZE = 4096;

// emulates a single physical memory, shared by all instances
static std::array<uint8_t, MAX_SIZE> _data{};

} // namespace

bool ConfigStorage::begin(size_t size)
{
  if (size > MAX_SIZE) return false;
  _size = size;
  return true;
}

size_t ConfigStorage::size() const
{
  return _size;
}

uint8_t ConfigStorage::readByte(size_t addr) const
{
  if (addr >= _size) return 0;
  return _data[addr];
}

void ConfigStorage::writeByte(size_t addr, uint8_t value)
{
  if (addr >= _size) return;
  _data[addr] = value;
}

size_t ConfigStorage::read(size_t addr, uint8_t* data, size_t len) const
{
  if (addr >= _size) return 0;
  len = std::min(len, _size - addr);
  std::memcpy(data, _data.data() + addr, len);
  return len;
}

size_t ConfigStorage::write(size_t addr, const uint8_t* data, size_t len)
{
  if (addr >= _size) return 0;
  len = std::min(len, _size - addr);
  std::memcpy(_data.data() + addr, data, len);
  return len;
}

bool ConfigStorage::commit()
{
  return true;
}

} // namespace Espfc::Hal

#endif
