#if defined(ESP32)

#include "Hal/Gpio.hpp"
#include "Hal/FastCode.hpp"
#include "hal/gpio_ll.h"
#include <Arduino.h>

namespace Espfc::Hal {

void FAST_CODE_ATTR Gpio::digitalWrite(uint8_t pin, Gpio::PinStatus val)
{
  //::gpio_set_level((gpio_num_t)pin, val);
  gpio_ll_set_level(&GPIO, (gpio_num_t)pin, val);
}

Gpio::PinStatus FAST_CODE_ATTR Gpio::digitalRead(uint8_t pin)
{
  // return ::gpio_get_level((gpio_num_t)pin);
  return ::gpio_ll_get_level(&GPIO, (gpio_num_t)pin) ? Gpio::PinStatus::High : Gpio::PinStatus::Low;
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
