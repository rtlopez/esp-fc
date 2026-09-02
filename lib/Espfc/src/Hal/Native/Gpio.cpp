#if defined(UNIT_TEST)

#include "Hal/Gpio.hpp"

namespace Espfc::Hal {

void Gpio::digitalWrite(uint8_t pin, pin_status_t val)
{
  // do nothing
}

pin_status_t Gpio::digitalRead(uint8_t pin)
{
  // do nothing
  return 0;
}

void FAST_CODE_ATTR Gpio::pinMode(uint8_t pin, pin_mode_t mode)
{
  // do nothing
  return;
}

} // namespace Espfc::Hal

#endif
