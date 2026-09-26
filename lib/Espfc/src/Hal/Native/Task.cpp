#if defined(UNIT_TEST)

#include "Hal/Task.hpp"

namespace Espfc::Hal {

bool Task::create(Function function, const char* name, size_t stackSize, void* arg, Priority priority, uint8_t core)
{
  return false;
}

Task::Handle Task::currentHandle()
{
  return nullptr;
}

bool Task::notifyFromIsr(Handle handle)
{
  return false;
}

void Task::waitNotify() {}

void Task::exitCurrent() {}

void Task::disableIdleWatchdog() {}

} // namespace Espfc::Hal

#endif
