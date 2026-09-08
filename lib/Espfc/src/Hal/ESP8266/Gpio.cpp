#if defined(ESP8266)

#include "Hal/Gpio.hpp"
#include "Utils/MemoryHelper.h"
#include <Arduino.h>

namespace Espfc::Hal {

void FAST_CODE_ATTR Gpio::digitalWrite(uint8_t pin, Gpio::PinStatus val)
{
  if (pin < 16)
  {
    if (val)
    {
      GPOS = (1 << pin);
    }
    else
    {
      GPOC = (1 << pin);
    }
  }
  else if (pin == 16)
  {
    if (val)
    {
      GP16O |= 1;
    }
    else
    {
      GP16O &= ~1;
    }
  }
}

Gpio::PinStatus FAST_CODE_ATTR Gpio::digitalRead(uint8_t pin)
{
  if (pin < 16)
  {
    return GPIP(pin) ? Gpio::High : Gpio::Low;
  }
  else if (pin == 16)
  {
    return (GP16I & 0x01) ? Gpio::High : Gpio::Low;
  }
  return Gpio::Low;
}

void FAST_CODE_ATTR Gpio::pinMode(uint8_t pin, Gpio::PinMode mode)
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
