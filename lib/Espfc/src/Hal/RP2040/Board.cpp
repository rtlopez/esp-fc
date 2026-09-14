#ifdef ARCH_RP2040

#include "Hal/Board.hpp"
#include <Arduino.h>
#include <RP2040Support.h>

namespace Espfc::Hal {

uint32_t Board::getId0()
{
  const char* id = rp2040.getChipID();
  return id[0] << 24 | id[1] << 16 | id[2] << 8 | id[3];
}

uint32_t Board::getId1()
{
  const char* id = rp2040.getChipID();
  return id[4] << 24 | id[5] << 16 | id[6] << 8 | id[7];
}

uint32_t Board::getId2()
{
  return 0;
}

void Board::reset()
{
  watchdog_enable(1, 1);
  while (1)
  {
  }
}

uint32_t Board::getCpuFreq()
{
  return rp2040.f_cpu() / 1000000u;
}

uint32_t Board::getFreeHeap()
{
  return rp2040.getFreeHeap();
}

} // namespace Espfc::Hal

#endif
