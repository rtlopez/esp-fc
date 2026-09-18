#if defined(ESP32)

#include "Hal/Flash.hpp"
#include "Hal/FastCode.hpp"
#include <algorithm>
#include <esp_partition.h>

namespace Espfc::Hal {

namespace {

const esp_partition_t* toPartition(FlashPartition partition)
{
  return reinterpret_cast<const esp_partition_t*>(partition);
}

} // namespace

FlashPartition Flash::findPartition()
{
  return esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, nullptr);
}

uint32_t FAST_CODE_ATTR Flash::getSize(FlashPartition partition)
{
  const auto p = toPartition(partition);
  return p ? p->size : 0;
}

uint32_t FAST_CODE_ATTR Flash::getSectorSize()
{
  return SPI_FLASH_SEC_SIZE;
}

size_t FAST_CODE_ATTR Flash::read(FlashPartition partition, uint32_t address, uint8_t* data, size_t len)
{
  const auto p = toPartition(partition);
  if (!p) return 0;

  len = std::min((uint32_t)len, p->size - address);
  if (esp_partition_read_raw(p, address, data, len) != ESP_OK) return 0;

  return len;
}

size_t FAST_CODE_ATTR Flash::write(FlashPartition partition, uint32_t address, const uint8_t* data, size_t len)
{
  const auto p = toPartition(partition);
  if (!p) return 0;

  if (esp_partition_write_raw(p, address, data, len) != ESP_OK) return 0;

  return len;
}

bool Flash::erase(FlashPartition partition, uint32_t address, uint32_t len)
{
  const auto p = toPartition(partition);
  if (!p) return false;

  return esp_partition_erase_range(p, address, len) == ESP_OK;
}

void Flash::printPartitions(Stream::Printer& s)
{
  s.printf("ESP32 Partition table:\r\n");
  s.printf("| Type | Sub |  Offset  |   Size   |       Label      |\r\n");
  s.printf("| ---- | --- | -------- | -------- | ---------------- |\r\n");

  esp_partition_iterator_t pi = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, nullptr);
  if (pi != NULL)
  {
    do
    {
      const esp_partition_t* p = esp_partition_get(pi);
      s.printf("|  %02x  | %02x  | 0x%06X | 0x%06X | %-16s |\r\n", p->type, p->subtype, p->address, p->size, p->label);
    } while ((pi = esp_partition_next(pi)));
  }
}

} // namespace Espfc::Hal

#endif
