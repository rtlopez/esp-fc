#pragma once

#include "Stream/Printer.hpp"
#include <cstddef>
#include <cstdint>

namespace Espfc::Hal {

// Opaque handle to a flash partition, owned by the caller.
using FlashPartition = const void*;

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

  static void printPartitions(Stream::Printer& s);
};

} // namespace Espfc::Hal
