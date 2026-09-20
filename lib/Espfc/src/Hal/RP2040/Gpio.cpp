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

static inline constexpr PinStatus toPinStatus(Gpio::InterruptMode mode)
{
  switch (mode)
  {
    case Gpio::InterruptMode::Rising:
      return RISING;
    case Gpio::InterruptMode::Falling:
      return FALLING;
    case Gpio::InterruptMode::Change:
      return CHANGE;
    default:
      return RISING;
  }
}

static_assert(toPinStatus(Gpio::InterruptMode::Rising) == RISING);
static_assert(toPinStatus(Gpio::InterruptMode::Falling) == FALLING);
static_assert(toPinStatus(Gpio::InterruptMode::Change) == CHANGE);

void Gpio::attachInterrupt(uint8_t pin, InterruptHandler handler, void* arg, InterruptMode mode)
{
  ::attachInterruptParam(pin, handler, toPinStatus(mode), arg);
}

void Gpio::detachInterrupt(uint8_t pin)
{
  ::detachInterrupt(pin);
}

} // namespace Espfc::Hal

#endif
