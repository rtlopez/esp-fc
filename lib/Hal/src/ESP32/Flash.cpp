#if defined(ESP32)

#include "Hal/Flash.hpp"
#include "Hal/FastCode.hpp"
#include <algorithm>
#include <esp_partition.h>

namespace Hal {

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

size_t Flash::read(FlashPartition partition, uint32_t address, uint8_t* data, size_t len)
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

size_t Flash::getPartitions(FlashPartitionInfo* list, size_t max)
{
  size_t count = 0;
  esp_partition_iterator_t pi = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, nullptr);
  while (pi && count < max)
  {
    const esp_partition_t* p = esp_partition_get(pi);
    list[count++] = {static_cast<uint8_t>(p->type), static_cast<uint8_t>(p->subtype), p->address, p->size, p->label};
    pi = esp_partition_next(pi);
  }
  esp_partition_iterator_release(pi);
  return count;
}

} // namespace Hal

#endif
