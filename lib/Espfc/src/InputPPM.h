#ifndef _INPUT_PPM_H_
#define _INPUT_PPM_H_

#include "Arduino.h"

namespace Espfc {

class InputPPM
{
  public:
    void begin(uint8_t pin);
    void handle() ICACHE_RAM_ATTR;

    uint16_t getTime(uint8_t i) const { return _channels[i * 2] + _channels[i * 2 + 1]; }
    unsigned long getStart() const { return _start; }
    boolean hasNewData() const { return _new_data; }
    boolean resetNewData() { _new_data = 0; }

    static const uint8_t CHANNELS     = 12;
    static const uint16_t TICK_SPACE  = 400;
    static const uint16_t TICK_MIN    = 500;  // with space 900
    static const uint16_t TICK_MAX    = 2000; // with space 2400
    static const uint16_t TICK_CENTER = 1100; // with space 1500
    static const uint16_t TICK_SYNC   = 3000;
    static const uint8_t THROTTLE_CHANNEL = 2;

  private:
    uint8_t _pin;
    volatile uint8_t  _channel;
    volatile uint8_t  _value;
    volatile boolean  _new_data;
    volatile unsigned long _start;
    volatile uint16_t _channels[CHANNELS * 2];
};

extern InputPPM PPM;

}

#endif
