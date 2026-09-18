#include "Target.h"

#if defined(ESPFC_FREE_RTOS_QUEUE)

#include "Queue.hpp"

namespace Espfc::Target {

void Queue::begin()
{
  _q = xQueueCreate(64, sizeof(Event));
}

void Queue::send(const Event& e)
{
  if (isFull()) return;
  xQueueSend(_q, &e, (TickType_t)0);
}

Event Queue::receive()
{
  Event e;
  xQueueReceive(_q, &e, portMAX_DELAY);
  return e;
}

bool Queue::isEmpty() const
{
  return uxQueueMessagesWaiting(_q) == 0;
}

bool Queue::isFull() const
{
  return uxQueueMessagesWaiting(_q) == 64;
}

} // namespace Espfc::Target

#endif
