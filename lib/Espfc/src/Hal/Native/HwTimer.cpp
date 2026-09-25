#if defined(UNIT_TEST)

#include "Hal/HwTimer.hpp"

namespace Espfc::Hal {

// no hardware timer on host
bool HwTimer::begin(uint32_t intervalUs, Callback callback, void* arg)
{
  return false;
}

void HwTimer::end() {}

} // namespace Espfc::Hal

#endif
