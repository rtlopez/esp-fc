#include "Utils/Storage.hpp"

namespace Espfc::Utils {

int Storage::begin()
{
  static_assert(sizeof(ModelConfig) <= EEPROM_SIZE, "ModelConfig Size too big");
  _initialized = _storage.begin(EEPROM_SIZE);
  return _initialized;
}

StorageResult Storage::load(ModelConfig& config) const
{
  if (!_initialized) return STORAGE_NONE;

  size_t addr = 0;
  uint8_t magic = _storage.readByte(addr++);
  if (EEPROM_MAGIC != magic)
  {
    return STORAGE_ERR_BAD_MAGIC;
  }

  uint8_t version = _storage.readByte(addr++);
  if (EEPROM_VERSION != version)
  {
    return STORAGE_ERR_BAD_VERSION;
  }

  uint16_t size = 0;
  size = _storage.readByte(addr++);
  size |= _storage.readByte(addr++) << 8;
  if (size != sizeof(ModelConfig))
  {
    return STORAGE_ERR_BAD_SIZE;
  }

  _storage.read(addr, reinterpret_cast<uint8_t*>(&config), sizeof(ModelConfig));
  return STORAGE_LOAD_SUCCESS;
}

StorageResult Storage::save(const ModelConfig& config)
{
  if (!_initialized) return STORAGE_SAVE_ERROR;

  size_t addr = 0;
  uint16_t size = sizeof(ModelConfig);
  _storage.writeByte(addr++, EEPROM_MAGIC);
  _storage.writeByte(addr++, EEPROM_VERSION);
  _storage.writeByte(addr++, size & 0xFF);
  _storage.writeByte(addr++, (size >> 8) & 0xFF);
  _storage.write(addr, reinterpret_cast<const uint8_t*>(&config), sizeof(ModelConfig));
  bool ok = _storage.commit();
  if (!ok) return STORAGE_SAVE_ERROR;
  return STORAGE_SAVE_SUCCESS;
}

} // namespace Espfc::Utils
