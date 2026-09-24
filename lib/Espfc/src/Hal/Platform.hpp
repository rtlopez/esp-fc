#pragma once

#include <cstddef>

#if defined(ESPFC_SINGLE_CORE)
#define ESPFC_HAL_CORE_COUNT 1
#elif defined(ARCH_RP2040) // covers RP2350 as well
#define ESPFC_HAL_CORE_COUNT 2
#elif defined(ESP32) && !defined(ESP32S2) && !defined(ESP32C3)
#define ESPFC_HAL_CORE_COUNT 2
#else
#define ESPFC_HAL_CORE_COUNT 1
#endif

namespace Espfc::Hal {

inline constexpr size_t CORE_COUNT = ESPFC_HAL_CORE_COUNT;
inline constexpr bool MULTI_CORE = CORE_COUNT > 1;

} // namespace Espfc::Hal
