#ifdef ARCH_RP2040

#include "Hal/ConfigStorage.hpp"
#include <EEPROM.h>
#include <algorithm>

namespace Espfc::Hal {

bool ConfigStorage::begin(size_t size)
{
  EEPROM.begin(size);
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
  return EEPROM.read(addr);
}

void ConfigStorage::writeByte(size_t addr, uint8_t value)
{
  if (addr >= _size) return;
  EEPROM.write(addr, value);
}

size_t ConfigStorage::read(size_t addr, uint8_t* data, size_t len) const
{
  if (addr >= _size) return 0;
  len = std::min(len, _size - addr);
  for (size_t i = 0; i < len; i++)
  {
    data[i] = EEPROM.read(addr + i);
  }
  return len;
}

size_t ConfigStorage::write(size_t addr, const uint8_t* data, size_t len)
{
  if (addr >= _size) return 0;
  len = std::min(len, _size - addr);
  for (size_t i = 0; i < len; i++)
  {
    EEPROM.write(addr + i, data[i]);
  }
  return len;
}

bool ConfigStorage::commit()
{
  return EEPROM.commit();
}

} // namespace Espfc::Hal

#endif
