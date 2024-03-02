#ifndef _ESP_GPIO_H_
#define _ESP_GPIO_H_

#include <Arduino.h>
#if defined(ESP32)
#include "hal/gpio_ll.h"
#endif

#if defined(ARCH_RP2040)
typedef PinStatus pin_status_t;
#else
typedef uint8_t pin_status_t;
#endif

class EspGpio
{
  public:
    static inline void IRAM_ATTR digitalWrite(uint8_t pin, pin_status_t val)
    {
#if defined(ESP8266)
      if(pin < 16)
      {
        if(val) GPOS = (1 << pin);
        else GPOC = (1 << pin);
      }
      else if(pin == 16)
      {
        if(val) GP16O |= 1;
        else GP16O &= ~1;
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

    static inline pin_status_t IRAM_ATTR digitalRead(uint8_t pin)
    {
#if defined(ESP8266)
      if(pin < 16)
      {
        return GPIP(pin);
      }
      else if(pin == 16)
      {
        return GP16I & 0x01;
      }
      return 0;
#elif defined(ESP32)
      //return ::gpio_get_level((gpio_num_t)pin);
      return ::gpio_ll_get_level(&GPIO, (gpio_num_t)pin);
#elif defined(UNIT_TEST)
      return 0;
      // do nothing
#else
      return ::digitalRead(pin);
#endif
    }
};

#endif // _ESP_GPIO_H_
