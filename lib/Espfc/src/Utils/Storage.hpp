#pragma once

#include "Hal/ConfigStorage.hpp"
#include "ModelConfig.h"

namespace Espfc {

enum StorageResult
{
  STORAGE_NONE,
  STORAGE_LOAD_SUCCESS,
  STORAGE_SAVE_SUCCESS,
  STORAGE_SAVE_ERROR,
  STORAGE_ERR_BAD_MAGIC,
  STORAGE_ERR_BAD_VERSION,
  STORAGE_ERR_BAD_SIZE,
};

namespace Utils {

class Storage
{
public:
  int begin();
  StorageResult load(ModelConfig& config) const;
  StorageResult save(const ModelConfig& config);

private:
  static constexpr uint8_t EEPROM_MAGIC = 0xA5;
  static constexpr uint8_t EEPROM_VERSION = 0x01;
  static constexpr size_t EEPROM_SIZE = 2048;

  Hal::ConfigStorage _storage;
  bool _initialized = false;
};

} // namespace Utils

} // namespace Espfc
