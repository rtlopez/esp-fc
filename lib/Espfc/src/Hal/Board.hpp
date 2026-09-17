#pragma once

#include <cstdint>

namespace Espfc::Hal {

enum class ResetReason
{
  UNKNOWN,
  POWER_ON,
  EXTERNAL_RESET,
  SOFTWARE,
  WATCHDOG,
  PANIC,
  BROWNOUT,
  DEEP_SLEEP,
  OTHER,
};

class Board
{
public:
  static uint32_t getId0();
  static uint32_t getId1();
  static uint32_t getId2();

  static void reset();
  static ResetReason getResetReason();

  static uint32_t getCpuFreq();
  static uint32_t getFreeHeap();
};

bool isUnexpectedReset(ResetReason reason);

} // namespace Espfc::Hal
