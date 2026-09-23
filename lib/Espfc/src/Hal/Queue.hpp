#pragma once

// Lock free queue used to pass events between cores.
//
// The default implementation is picked from the platform properties: multi core targets get
// QueueAtomic, single core targets get QueueNull, as they dispatch events inline.
//
// Reference implementations backed by the platform SDK can be selected at build time, to compare
// them against QueueAtomic:
//   -DESPFC_HAL_QUEUE_ATOMIC    force the atomic implementation
//   -DESPFC_HAL_QUEUE_FREERTOS  FreeRTOS queue, ESP32 only
//   -DESPFC_HAL_QUEUE_PICOSDK   pico-sdk queue, RP2040/RP2350 only

#include "Hal/Detail/QueueAtomic.hpp"
#include "Hal/Detail/QueueNull.hpp"
#include "Hal/Platform.hpp"
#include <cstddef>
#include <type_traits>

#if defined(ESPFC_HAL_QUEUE_FREERTOS)
#include "Hal/Detail/QueueFreeRTOS.hpp"
#elif defined(ESPFC_HAL_QUEUE_PICOSDK)
#include "Hal/Detail/QueuePicoSdk.hpp"
#endif

namespace Espfc::Hal {

#if defined(ESPFC_HAL_QUEUE_FREERTOS)
template<typename T, size_t Capacity>
using Queue = Detail::QueueFreeRTOS<T, Capacity>;
#elif defined(ESPFC_HAL_QUEUE_PICOSDK)
template<typename T, size_t Capacity>
using Queue = Detail::QueuePicoSdk<T, Capacity>;
#elif defined(ESPFC_HAL_QUEUE_ATOMIC)
template<typename T, size_t Capacity>
using Queue = Detail::QueueAtomic<T, Capacity>;
#else
template<typename T, size_t Capacity>
using Queue = std::conditional_t<MULTI_CORE, Detail::QueueAtomic<T, Capacity>, Detail::QueueNull<T, Capacity>>;
#endif

} // namespace Espfc::Hal
