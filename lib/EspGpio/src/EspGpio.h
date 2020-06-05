#ifndef _ESP_GPIO_H_
#define _ESP_GPIO_H_

#include <Arduino.h>

class EspGpio
{
  public:
    static inline void digitalWrite(uint8_t pin, uint8_t val)
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
#elif defined(UNIT_TEST)
      // do nothing
#else
      ::digitalWrite(pin, val);
#endif
    }

    static inline int digitalRead(uint8_t pin) 
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
#elif defined(UNIT_TEST)
      return 0;
      // do nothing
#else
      return ::digitalRead(pin);
#endif
    }
};

#endif // _ESP_GPIO_H_
