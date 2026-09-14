#pragma once

#include <cstdint>

namespace Espfc::Hal {

class Board
{
public:
  static uint32_t getId0();
  static uint32_t getId1();
  static uint32_t getId2();

  static void reset();

  static uint32_t getCpuFreq();
  static uint32_t getFreeHeap();
};

} // namespace Espfc::Hal
