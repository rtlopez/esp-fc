#if defined(ARCH_RP2040)

#include "Hal/HwTimer.hpp"

namespace Espfc::Hal {

// TODO: implement with hardware_alarm
bool HwTimer::begin(uint32_t intervalUs, Callback callback, void* arg)
{
  return false;
}

void HwTimer::end() {}

} // namespace Espfc::Hal

#endif
