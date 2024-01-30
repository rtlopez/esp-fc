#ifndef _ESPFC_STORAGE_H_
#define _ESPFC_STORAGE_H_

namespace Espfc {

enum StorageResult
{
  STORAGE_NONE,
  STORAGE_LOAD_SUCCESS,
  STORAGE_SAVE_SUCCESS,
  STORAGE_ERR_BAD_MAGIC,
  STORAGE_ERR_BAD_VERSION,
  STORAGE_ERR_BAD_SIZE,
};

}

#ifndef UNIT_TEST

#include <Arduino.h>
#include "ModelConfig.h"
#include "EEPROM.h"
#include "Logger.h"

namespace Espfc {

class Storage
{
  public:
    int begin()
    {
      EEPROM.begin(EEPROM_SIZE);
      static_assert(sizeof(ModelConfig) <= EEPROM_SIZE, "ModelConfig Size too big");
      return 1;
    }

    StorageResult load(ModelConfig& config)
    {
      //return STORAGE_ERR_BAD_MAGIC;

      int addr = 0;
      uint8_t magic = EEPROM.read(addr++);
      if(EEPROM_MAGIC != magic)
      {
        return STORAGE_ERR_BAD_MAGIC;
      }

      uint8_t version = EEPROM.read(addr++);
      if(EEPROM_VERSION != version)
      {
        return STORAGE_ERR_BAD_VERSION;
      }

      uint16_t size = 0;
      size = EEPROM.read(addr++);
      size |= EEPROM.read(addr++) << 8;
      if(size != sizeof(ModelConfig))
      {
        return STORAGE_ERR_BAD_SIZE;
      }

      uint8_t * begin = reinterpret_cast<uint8_t*>(&config);
      uint8_t * end = begin + size;
      for(uint8_t * it = begin; it < end; ++it)
      {
        *it = EEPROM.read(addr++);
      }
      return STORAGE_LOAD_SUCCESS;
    }

    StorageResult write(const ModelConfig& config)
    {
      int addr = 0;
      uint16_t size = sizeof(ModelConfig);
      EEPROM.write(addr++, EEPROM_MAGIC);
      EEPROM.write(addr++, EEPROM_VERSION);
      EEPROM.write(addr++, size & 0xFF);
      EEPROM.write(addr++, size >> 8);
      const uint8_t * begin = reinterpret_cast<const uint8_t*>(&config);
      const uint8_t * end = begin + sizeof(ModelConfig);
      for(const uint8_t * it = begin; it < end; ++it)
      {
        EEPROM.write(addr++, *it);
      }
      EEPROM.commit();
      return STORAGE_SAVE_SUCCESS;
    }

  private:
    static const uint8_t EEPROM_MAGIC   = 0xA5;
    static const uint8_t EEPROM_VERSION = 0x01;
    static const size_t  EEPROM_SIZE    = 2048;
#if defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_EEPROM)
    EEPROMClass EEPROM;
#endif
};

}
#endif

#endif