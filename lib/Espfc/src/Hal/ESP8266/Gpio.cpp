#if defined(ESP8266)

#include "Hal/Gpio.hpp"
#include "Hal/FastCode.hpp"
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

static_assert(Gpio::InterruptMode::Rising == RISING);
static_assert(Gpio::InterruptMode::Falling == FALLING);
static_assert(Gpio::InterruptMode::Change == CHANGE);

void Gpio::attachInterrupt(uint8_t pin, InterruptHandler handler, void* arg, InterruptMode mode)
{
  ::attachInterruptArg(pin, handler, arg, mode);
}

void Gpio::detachInterrupt(uint8_t pin)
{
  ::detachInterrupt(pin);
}

} // namespace Espfc::Hal

#endif
