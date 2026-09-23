#if defined(ESP32)

#include "Hal/Detail/QueueFreeRTOS.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace Espfc::Hal::Detail {

static QueueHandle_t handle(void* h)
{
  return static_cast<QueueHandle_t>(h);
}

void QueueFreeRtosBase::beginImpl()
{
  _handle = xQueueCreate(_count, _elemSize);
}

bool QueueFreeRtosBase::pushBytes(const void* src)
{
  return xQueueSendToBack(handle(_handle), src, (TickType_t)0) == pdTRUE;
}

bool QueueFreeRtosBase::popBytes(void* dst)
{
  return xQueueReceive(handle(_handle), dst, (TickType_t)0) == pdTRUE;
}

size_t QueueFreeRtosBase::sizeImpl() const
{
  return uxQueueMessagesWaiting(handle(_handle));
}

} // namespace Espfc::Hal::Detail

#endif
