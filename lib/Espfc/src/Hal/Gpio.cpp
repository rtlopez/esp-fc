#include <Arduino.h>
#include "Gpio.h"
#include "Utils/MemoryHelper.h"
#if defined(ESP32)
#include "hal/gpio_ll.h"
#endif

namespace Espfc {

namespace Hal {

void FAST_CODE_ATTR Gpio::digitalWrite(uint8_t pin, pin_status_t val)
{
#if defined(ESP8266)
  if (pin < 16)
  {
    if (val)
      GPOS = (1 << pin);
    else
      GPOC = (1 << pin);
  }
  else if (pin == 16)
  {
    if (val)
      GP16O |= 1;
    else
      GP16O &= ~1;
  }
#elif defined(ESP32)
  //::gpio_set_level((gpio_num_t)pin, val);
  gpio_ll_set_level(&GPIO, (gpio_num_t)pin, val);
#elif defined(UNIT_TEST)
  // do nothing
#else
  ::digitalWrite(pin, val);
#endif
}

pin_status_t FAST_CODE_ATTR Gpio::digitalRead(uint8_t pin)
{
#if defined(ESP8266)
  if (pin < 16)
  {
    return GPIP(pin);
  }
  else if (pin == 16)
  {
    return GP16I & 0x01;
  }
  return 0;
#elif defined(ESP32)
  // return ::gpio_get_level((gpio_num_t)pin);
  return ::gpio_ll_get_level(&GPIO, (gpio_num_t)pin);
#elif defined(UNIT_TEST)
  return 0;
  // do nothing
#else
  return ::digitalRead(pin);
#endif
}

}

}
