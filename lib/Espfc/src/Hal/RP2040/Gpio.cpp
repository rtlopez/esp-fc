#if defined(ARCH_RP2040)

#include "Hal/Gpio.hpp"
#include <Arduino.h>

namespace Espfc::Hal {

void Gpio::digitalWrite(uint8_t pin, Gpio::PinStatus val)
{
  ::digitalWrite(pin, val == Gpio::High ? HIGH : LOW);
}

Gpio::PinStatus Gpio::digitalRead(uint8_t pin)
{
  return ::digitalRead(pin) ? Gpio::High : Gpio::Low;
}

void Gpio::pinMode(uint8_t pin, Gpio::PinMode mode)
{
  switch (mode)
  {
    case Gpio::Input:
      ::pinMode(pin, INPUT);
      break;
    case Gpio::InputPullup:
      ::pinMode(pin, INPUT_PULLUP);
      break;
    case Gpio::Output:
      ::pinMode(pin, OUTPUT);
      break;
  }
}

} // namespace Espfc::Hal

#endif
