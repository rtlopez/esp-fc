#if defined(ESP32)

#include "Hal/Gpio.hpp"
#include "Utils/MemoryHelper.h"
#include "hal/gpio_ll.h"
#include <Arduino.h>

namespace Espfc::Hal {

void FAST_CODE_ATTR Gpio::digitalWrite(uint8_t pin, pin_status_t val)
{
  //::gpio_set_level((gpio_num_t)pin, val);
  gpio_ll_set_level(&GPIO, (gpio_num_t)pin, val);
}

pin_status_t FAST_CODE_ATTR Gpio::digitalRead(uint8_t pin)
{
  // return ::gpio_get_level((gpio_num_t)pin);
  return ::gpio_ll_get_level(&GPIO, (gpio_num_t)pin);
}

void FAST_CODE_ATTR Gpio::pinMode(uint8_t pin, pin_mode_t mode)
{
  ::pinMode(pin, mode);
}

} // namespace Espfc::Hal

#endif
