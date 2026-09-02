#if defined(ARCH_RP2040)

#include "Hal/Gpio.hpp"
#include <Arduino.h>

namespace Espfc::Hal {

void Gpio::digitalWrite(uint8_t pin, pin_status_t val)
{
  ::digitalWrite(pin, val);
}

pin_status_t Gpio::digitalRead(uint8_t pin)
{
  return ::digitalRead(pin);
}

void Gpio::pinMode(uint8_t pin, pin_mode_t mode)
{
  ::pinMode(pin, mode);
}

} // namespace Espfc::Hal

#endif
