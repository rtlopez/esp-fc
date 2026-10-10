#pragma once

#include <cstddef>

#if defined(ESPFC_SINGLE_CORE) || defined(ESP32S2) || defined(ESP32C3) || defined(ESP8266) || defined(UNIT_TEST)
#define ESPFC_HAL_CORE_COUNT 1
#else
#define ESPFC_HAL_CORE_COUNT 2
#endif

#if defined(ARDUINO_USB_CDC_ON_BOOT) || defined(ARCH_RP2040)
#define ESPFC_HAL_USB_CDC
#endif

#if defined(ESP32)
#define ESPFC_HAL_LED_WS2812
#define ESPFC_HAL_ESPNOW
#define ESPFC_HAL_FREE_RTOS
#define ESPFC_HAL_DSP
#endif

#if defined(ARCH_RP2040)
#if defined(__FREERTOS)
#define ESPFC_HAL_FREE_RTOS
#endif
#define ESPFC_HAL_MULTI_CORE_RP2040
#define ESPFC_HAL_C_LINKAGE_ENTRY
#endif

#if defined(ARCH_RP2350)
#define ESPFC_HAL_DSP
#endif

#if defined(ESP8266)
#define ESPFC_HAL_C_LINKAGE_ENTRY
#endif

namespace Espfc::Hal {

inline constexpr size_t CORE_COUNT = ESPFC_HAL_CORE_COUNT;
inline constexpr bool MULTI_CORE = CORE_COUNT > 1;

} // namespace Espfc::Hal
