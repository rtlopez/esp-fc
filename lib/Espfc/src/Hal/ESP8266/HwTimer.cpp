#if defined(ESP8266)

#include "Hal/HwTimer.hpp"

namespace Espfc::Hal {

// not implemented, single core target does not use it
bool HwTimer::begin(uint32_t intervalUs, Callback callback, void* arg)
{
  return false;
}

void HwTimer::end() {}

} // namespace Espfc::Hal

#endif
