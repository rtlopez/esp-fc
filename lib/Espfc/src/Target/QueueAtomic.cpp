#include "Target.h"

#if defined(ESPFC_ATOMIC_QUEUE)

#include "Queue.h"

namespace Espfc {

namespace Target {

void Queue::begin()
{
}

void Queue::send(const Event& e)
{
  if(isFull()) return;
  _q.push(e);
}

Event Queue::receive()
{
  Event e;
  _q.pop(e);
  return e;
}

bool Queue::isEmpty() const
{
  return _q.isEmpty();
}

bool Queue::isFull() const
{
  return _q.isFull();
}

}

}

#endif
