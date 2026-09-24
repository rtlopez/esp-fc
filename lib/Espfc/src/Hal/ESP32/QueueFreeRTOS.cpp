#if defined(ESP32)

#include "Hal/Detail/QueueFreeRTOS.hpp"
#include "Hal/FastCode.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace Espfc::Hal::Detail {

static inline QueueHandle_t handle(void* h)
{
  return static_cast<QueueHandle_t>(h);
}

void FAST_CODE_ATTR QueueFreeRtosBase::beginImpl()
{
  _handle = xQueueCreate(_count, _elemSize);
}

bool FAST_CODE_ATTR QueueFreeRtosBase::pushBytes(const void* src)
{
  return xQueueSendToBack(handle(_handle), src, (TickType_t)0) == pdTRUE;
}

bool FAST_CODE_ATTR QueueFreeRtosBase::popBytes(void* dst)
{
  return xQueueReceive(handle(_handle), dst, (TickType_t)0) == pdTRUE;
}

size_t FAST_CODE_ATTR QueueFreeRtosBase::sizeImpl() const
{
  return uxQueueMessagesWaiting(handle(_handle));
}

} // namespace Espfc::Hal::Detail

#endif
