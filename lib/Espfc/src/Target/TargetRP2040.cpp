#ifdef ARCH_RP2040

#include "TargetRP2040.h"
#include <Arduino.h>
#include <RP2040Support.h>

namespace Espfc {

uint32_t getBoardId0()
{
  const char* id = rp2040.getChipID();
  return id[0] << 24 | id[1] << 16 | id[2] << 8 | id[3];
}

uint32_t getBoardId1()
{
  const char* id = rp2040.getChipID();
  return id[4] << 24 | id[5] << 16 | id[6] << 8 | id[7];
}

uint32_t getBoardId2()
{
  return 0;
}

void targetReset()
{
  watchdog_enable(1, 1);
  while (1)
  {
  }
}

uint32_t targetCpuFreq()
{
  return rp2040.f_cpu() / 1000000u;
}

uint32_t targetFreeHeap()
{
  return rp2040.getFreeHeap();
}

} // namespace Espfc

#endif
