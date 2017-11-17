#ifndef _ESP_GPIO_H_
#define _ESP_GPIO_H_

#include "Arduino.h"

class EspGpio
{
  public:
    static inline void digitalWrite(uint8_t pin, uint8_t val) ICACHE_RAM_ATTR
    {
      //::digitalWrite(pin, val);
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
    }

    static inline int digitalRead(uint8_t pin) ICACHE_RAM_ATTR
    {
      //return ::digitalRead(pin);
      if(pin < 16)
      {
        return GPIP(pin);
      }
      else if(pin == 16)
      {
        return GP16I & 0x01;
      }
      return 0;
    }
};

#endif // _ESP_GPIO_H_
