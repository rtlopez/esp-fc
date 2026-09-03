#if defined(UNIT_TEST)

#include "Hal/Gpio.hpp"

namespace Espfc::Hal {

void Gpio::digitalWrite(uint8_t pin, Gpio::PinStatus val)
{
  // do nothing
}

Gpio::PinStatus Gpio::digitalRead(uint8_t pin)
{
  // do nothing
  return Gpio::Low;
}

void Gpio::pinMode(uint8_t pin, Gpio::PinMode mode)
{
  // do nothing
  return;
}

} // namespace Espfc::Hal

#endif
