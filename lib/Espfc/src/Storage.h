#ifndef _ESPFC_STORAGE_H_
#define _ESPFC_STORAGE_H_

#include <Arduino.h>
#include "ModelConfig.h"
#include "EEPROM.h"
#include "Logger.h"

#define ESP_STATIC_ASSERT(condition, name) \
    typedef char assert_failed_ ## name [(condition) ? 1 : -1 ] __attribute__((unused))

namespace Espfc {

class Storage
{
  public:
    int begin()
    {
      EEPROM.begin(EEPROM_SIZE);
      ESP_STATIC_ASSERT(sizeof(ModelConfig) <= EEPROM_SIZE, eeprom_size);
      return 1;
    }

    int load(ModelConfig& config, Logger& logger)
    {
      int addr = 0;
      uint8_t magic = EEPROM.read(addr++);
      if(EEPROM_MAGIC != magic)
      {
        logger.err().logln(F("EEPROM bad magic"));
        return -1;
      }

      uint8_t version = EEPROM.read(addr++);
      if(EEPROM_VERSION != version)
      {
        logger.err().log(F("EEPROM bad version")).logln(version);
        return -1;
      }

      uint16_t size = 0;
      size = EEPROM.read(addr++);
      size |= EEPROM.read(addr++) << 8;
      if(size != sizeof(ModelConfig))
      {
        logger.err().log(F("EEPROM bad size")).log(size).logln(sizeof(ModelConfig));
        return -1;
      }

      uint8_t * begin = reinterpret_cast<uint8_t*>(&config);
      uint8_t * end = begin + size;
      for(uint8_t * it = begin; it < end; ++it)
      {
        *it = EEPROM.read(addr++);
      }
      logger.info().log(F("EEPROM loaded")).logln(size);
      return 1;
    }

    void write(const ModelConfig& config, Logger& logger)
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
      logger.info().logln(F("EEPROM saved"));
    }

  private:
    static const uint8_t EEPROM_MAGIC   = 0xA5;
    static const uint8_t EEPROM_VERSION = 0x00;
    static const size_t  EEPROM_SIZE    = 2048;
};

}

#endif