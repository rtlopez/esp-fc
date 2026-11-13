#pragma once

#include <cstddef>
#include <cstdint>

namespace Hal {

// Opaque handle to a flash partition, owned by the caller.
using FlashPartition = const void*;

struct FlashPartitionInfo
{
  uint8_t type;
  uint8_t subtype;
  uint32_t address;
  uint32_t size;
  const char* label;
};

class Flash
{
public:
  static FlashPartition findPartition();

  // Raw partition size, without any filesystem reservation.
  static uint32_t getSize(FlashPartition partition);
  static uint32_t getSectorSize();

  static size_t read(FlashPartition partition, uint32_t address, uint8_t* data, size_t len);
  static size_t write(FlashPartition partition, uint32_t address, const uint8_t* data, size_t len);
  static bool erase(FlashPartition partition, uint32_t address, uint32_t len);

  // Fills up to max entries, returns the number of entries written.
  static size_t getPartitions(FlashPartitionInfo* list, size_t max);
};

} // namespace Hal
