#pragma once

#include <cstddef>
#include <cstdint>

#define ESPFC_WIFI
#define ESPFC_ESPNOW
#define ESPFC_LED_WS2812

namespace Espfc {

constexpr size_t targetSerialTxBufferSize()
{
  return 0xFF;
}

}; // namespace Espfc
