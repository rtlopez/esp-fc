#if defined(ESP32)

#include "Hal/Board.hpp"
#include <Esp.h>

namespace Espfc::Hal {

uint32_t Board::getId0()
{
  const int64_t mac = ESP.getEfuseMac();
  return (uint32_t)mac;
}

uint32_t Board::getId1()
{
  const int64_t mac = ESP.getEfuseMac();
  return (uint32_t)(mac >> 32);
}

uint32_t Board::getId2()
{
  return 0;
}

void Board::reset()
{
  ESP.restart();
  while (1)
  {
  }
}

uint32_t Board::getCpuFreq()
{
  return ESP.getCpuFreqMHz();
}

uint32_t Board::getFreeHeap()
{
  return ESP.getFreeHeap();
}

} // namespace Espfc::Hal

#endif
