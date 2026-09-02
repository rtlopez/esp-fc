#if defined(ESP8266)

#include "Hal/Gpio.hpp"
#include "Utils/MemoryHelper.h"
#include <Arduino.h>

namespace Espfc::Hal {

void FAST_CODE_ATTR Gpio::digitalWrite(uint8_t pin, pin_status_t val)
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

pin_status_t FAST_CODE_ATTR Gpio::digitalRead(uint8_t pin)
{
  if (pin < 16)
  {
    return GPIP(pin);
  }
  else if (pin == 16)
  {
    return GP16I & 0x01;
  }
  return 0;
}

void FAST_CODE_ATTR Gpio::pinMode(uint8_t pin, pin_mode_t mode)
{
  ::pinMode(pin, mode);
}

} // namespace Espfc::Hal

#endif
