#if defined(ESP32)

#include "Hal/Board.hpp"
#include "Hal/Platform.hpp"
#include <Esp.h>
#include <esp_system.h>
#include <sdkconfig.h>
#include <soc/soc_caps.h>

#if defined(CONFIG_FREERTOS_UNICORE)
static_assert(!Espfc::Hal::MULTI_CORE, "sdkconfig is unicore, build with -DESPFC_SINGLE_CORE");
#else
static_assert(Espfc::Hal::CORE_COUNT == SOC_CPU_CORES_NUM, "Hal::CORE_COUNT does not match SoC core count");
#endif

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

ResetReason Board::getResetReason()
{
  switch (esp_reset_reason())
  {
    case ESP_RST_POWERON:
      return ResetReason::POWER_ON;
    case ESP_RST_EXT:
      return ResetReason::EXTERNAL_RESET;
    case ESP_RST_SW:
      return ResetReason::SOFTWARE;
    case ESP_RST_PANIC:
      return ResetReason::PANIC;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      return ResetReason::WATCHDOG;
    case ESP_RST_DEEPSLEEP:
      return ResetReason::DEEP_SLEEP;
    case ESP_RST_BROWNOUT:
      return ResetReason::BROWNOUT;
    case ESP_RST_UNKNOWN:
      return ResetReason::UNKNOWN;
    default:
      return ResetReason::OTHER;
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
