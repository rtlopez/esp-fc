#include "Target.h"

#if defined(UNIT_TEST) || !defined(ESPFC_MULTI_CORE)

#include "Queue.h"

namespace Espfc {

namespace Target {

void Queue::begin() {}

void FAST_CODE_ATTR Queue::send(const Event& e) { (void)e; }

Event FAST_CODE_ATTR Queue::receive() { return Event(); }

bool FAST_CODE_ATTR Queue::isEmpty() const { return true; }

bool FAST_CODE_ATTR Queue::isFull() const { return false; }

}

}

#endif