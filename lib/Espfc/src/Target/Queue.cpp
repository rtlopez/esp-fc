#include "Target.h"

#if defined(UNIT_TEST) || !defined(ESPFC_MULTI_CORE)

#include "Queue.h"

namespace Espfc {

namespace Target {

void Queue::begin() {}

void Queue::send(const Event& e) { (void)e; }

Event Queue::receive() { return Event(); }

bool Queue::isEmpty() const { return true; }

bool Queue::isFull() const { return false; }

}

}

#endif