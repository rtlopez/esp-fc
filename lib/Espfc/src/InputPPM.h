#ifndef _INPUT_PPM_H_
#define _INPUT_PPM_H_

#include "Arduino.h"

namespace Espfc {

class InputPPM
{
  public:
    void begin(uint8_t pin, int mode = RISING);
    void handle() ICACHE_RAM_ATTR;

    uint16_t getTime(uint8_t i) const { return _channels[i]; }
    uint32_t getStart() const { return _start; }
    bool hasNewData() const { return _new_data; }
    bool resetNewData() { _new_data = false; }

    static const uint8_t CHANNELS = 16;

  private:
    volatile uint16_t _channels[CHANNELS];
    volatile uint32_t _start;
    volatile uint8_t  _channel;
    volatile bool     _new_data;
    uint8_t _pin;
};

extern InputPPM PPM;

}

#endif
