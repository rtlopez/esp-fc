#ifndef _INPUT_PPM_H_
#define _INPUT_PPM_H_

#include "Arduino.h"

namespace Espfc {

class InputPPM
{
  public:
    void begin(uint8_t pin, int mode = RISING);
    void handle() ICACHE_RAM_ATTR;

    bool fail()
    {
      // sometimes micros() is lower than _last_tick,
      // probably ISR occured before comparision,
      // small time compensation allows to avoid false detection
      uint32_t now = micros() + 1000;
      uint32_t delta = now - _last_tick;
      return delta > BROKEN_LINK_US;
    }

    uint16_t getPulse(uint8_t i) const { return _channels[i]; }
    bool hasNewData() const { return _new_data; }
    void resetNewData() { _new_data = false; }

    static const size_t CHANNELS = 16;
    static const uint32_t BROKEN_LINK_US = 100000UL; // 100ms

  private:
    volatile uint16_t _channels[CHANNELS];
    volatile uint32_t _last_tick;
    volatile uint8_t  _channel;
    volatile bool     _new_data;
    uint8_t _pin;
};

extern InputPPM PPM;

}

#endif
