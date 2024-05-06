#pragma once

#include <Arduino.h>
#include <platform.h>
#include <algorithm>
#include "esp_partition.h"

namespace Espfc {

namespace Device {

class FlashPartition
{
public:
  FlashPartition(esp_partition_t* p): _p(p) {}

  size_t read(uint32_t address, uint8_t * data, size_t len)
  {
    len = std::min((uint32_t)len, _p->size - address);
    if(esp_partition_read_raw(_p, address, data, len) == ESP_OK)
    {
      return len;
    }
    return 0;
  }

  size_t write(uint32_t address, const uint8_t * data, size_t len)
  {
    esp_partition_write_raw(_p, address, data, len);
    return len;
  }

private:
  esp_partition_t* _p;
};

class FlashDevice
{ 
public:
  static void partitions(Stream& s)
  {
    s.printf("ESP32 Partition table:\r\n");
    s.printf("| Type | Sub |  Offset  |   Size   |       Label      |\r\n");
    s.printf("| ---- | --- | -------- | -------- | ---------------- |\r\n");
    
    esp_partition_iterator_t pi = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, nullptr);
    if (pi != NULL) {
      do {
        const esp_partition_t* p = esp_partition_get(pi);
        s.printf("|  %02x  | %02x  | 0x%06X | 0x%06X | %-16s |\r\n", 
          p->type, p->subtype, p->address, p->size, p->label);
      } while ((pi = esp_partition_next(pi)));
    }
  }

  static const esp_partition_t* findPartition()
  {
    return esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, nullptr);
  }

  static void journal(Stream& s)
  {
    FlashfsJournalItem journal[8];
    flashfsJournalLoad(journal, 0, 8);
    for(size_t i = 0; i < 8; i++)
    {
      const auto& it = journal[i];
      s.printf("%08x => %08x\r\n", it.logBegin, it.logEnd);
    }
  }

};

}

}