#ifndef _INPUT_PPM_H_
#define _INPUT_PPM_H_

#include "Arduino.h"

namespace Espfc {

class InputPPM
{
  public:
    void begin(uint8_t pin);
    void handle() ICACHE_RAM_ATTR;

    uint16_t getTime(uint8_t i) const { return _channels[i * 2] + _channels[(i * 2) + 1]; }
    unsigned long getStart() const { return _start; }
    boolean hasNewData() const { return _new_data; }
    boolean resetNewData() { _new_data = false; }

    static const uint8_t CHANNELS     =   16;
    static const uint16_t TICK_SPACE  =  400;
    static const uint16_t TICK_MIN    =  500; // with space  900
    static const uint16_t TICK_CENTER = 1100; // with space 1500
    static const uint16_t TICK_MAX    = 1700; // with space 2100
    static const uint16_t TICK_SYNC   = 3000;
    static const uint8_t THROTTLE_CHANNEL = 2;

  private:
    volatile uint16_t _channels[CHANNELS * 2];
    volatile unsigned long _start;
    volatile uint8_t  _channel;
    volatile boolean  _new_data;
    uint8_t _pin;
};

extern InputPPM PPM;

}

#endif
