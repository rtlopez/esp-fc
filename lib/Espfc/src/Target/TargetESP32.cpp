#if defined(ESP32)

#include <Esp.h>

namespace Espfc {

uint32_t getBoardId0()
{
  const int64_t mac = ESP.getEfuseMac();
  return (uint32_t)mac;
}

uint32_t getBoardId1()
{
  const int64_t mac = ESP.getEfuseMac();
  return (uint32_t)(mac >> 32);
}

uint32_t getBoardId2()
{
  return 0;
}

void targetReset()
{
  ESP.restart();
  while (1)
  {
  }
}

uint32_t targetCpuFreq()
{
  return ESP.getCpuFreqMHz();
}

uint32_t targetFreeHeap()
{
  return ESP.getFreeHeap();
}

} // namespace Espfc

#endif
