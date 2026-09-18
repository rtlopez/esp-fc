#include "Target.h"

#if defined(UNIT_TEST) || !defined(ESPFC_MULTI_CORE)

#include "Hal/FastCode.hpp"
#include "Queue.hpp"

namespace Espfc::Target {

void Queue::begin() {}

void FAST_CODE_ATTR Queue::send(const Event& e)
{
  (void)e;
}

Event FAST_CODE_ATTR Queue::receive()
{
  return Event();
}

bool FAST_CODE_ATTR Queue::isEmpty() const
{
  return true;
}

bool FAST_CODE_ATTR Queue::isFull() const
{
  return false;
}

} // namespace Espfc::Target

#endif