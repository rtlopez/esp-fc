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

void Gpio::attachInterrupt(uint8_t pin, InterruptHandler handler, void* arg, InterruptMode mode) {}

void Gpio::detachInterrupt(uint8_t pin) {}

bool Gpio::attachCaptureInterrupt(uint8_t pin, CaptureHandler handler, void* arg, InterruptMode mode)
{
  return false;
}

void Gpio::detachCaptureInterrupt(uint8_t pin) {}

} // namespace Espfc::Hal

#endif
