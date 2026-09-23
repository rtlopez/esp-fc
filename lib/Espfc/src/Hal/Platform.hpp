#pragma once

#include <cstddef>

namespace Espfc::Hal {

#if defined(ARCH_RP2040) // covers RP2350 as well
inline constexpr size_t CORE_COUNT = 2;
#elif defined(ESP32) && !defined(ESP32S2) && !defined(ESP32C3)
inline constexpr size_t CORE_COUNT = 2;
#else
inline constexpr size_t CORE_COUNT = 1;
#endif

inline constexpr bool MULTI_CORE = CORE_COUNT > 1;

} // namespace Espfc::Hal
