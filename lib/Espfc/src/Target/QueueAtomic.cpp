#include "Target.h"

#if defined(ESPFC_ATOMIC_QUEUE)

#include "Queue.h"

namespace Espfc {

namespace Target {

void Queue::begin()
{
}

void FAST_CODE_ATTR Queue::send(const Event& e)
{
  if(isFull()) return;
  _q.push(e);
}

Event FAST_CODE_ATTR Queue::receive()
{
  Event e;
  _q.pop(e);
  return e;
}

bool FAST_CODE_ATTR Queue::isEmpty() const
{
  return _q.isEmpty();
}

bool FAST_CODE_ATTR Queue::isFull() const
{
  return _q.isFull();
}

}

}

#endif
