#ifndef _OUTPUT_PWM_FAST_H_
#define _OUTPUT_PWM_FAST_H_

#include "Model.h"
#include <algorithm>

namespace Espfc {

class OutputPWMFast
{
  public:
    class Slot
    {
      public:
        Slot(): pin(-1), pulse(1000), diff(0) {}
        volatile uint32_t pulse;
        volatile uint32_t diff;
        volatile uint8_t pin;
        bool operator<(const Slot& rhs) const { return this->pulse < rhs.pulse; }
    };

    OutputPWMFast();
    int begin(int rate, OutputProtocol protocol);
    int update();

    inline uint32_t usToTicks(uint32_t us) const ICACHE_RAM_ATTR
    {
      //uint32_t ticks = microsecondsToClockCycles(us); // timer0
      uint32_t ticks = APB_CLK_FREQ / 1000000L * us; // timer1
      //uint32_t ticks = F_CPU / 1000000L / 2 * us; // timer1

      switch(_protocol)
      {
        case OUTPUT_ONESHOT125:
          ticks = ticks >> 3;
          break;
        default:
          break;
      }
      return ticks - 180; // ~180 cycles compensation for isr trigger
    }

    int attach(int slot_id, int pin, int pulse)
    {
      if(slot_id < 0 || slot_id >= OUTPUT_CHANNELS) return 0;
      _buffer[slot_id].pin = pin;
      _buffer[slot_id].pulse = usToTicks(pulse);
      pinMode(pin, OUTPUT);
      return 1;
    }

    int write(int slot_id, int pulse) ICACHE_RAM_ATTR
    {
      if(slot_id < 0 || slot_id >= OUTPUT_CHANNELS) return 0;
      _buffer[slot_id].pulse = usToTicks(pulse);
      return 1;
    }

    void apply() ICACHE_RAM_ATTR
    {
      Slot tmp[OUTPUT_CHANNELS];
      std::copy(_buffer, _buffer + OUTPUT_CHANNELS, tmp);
      std::sort(tmp, tmp + OUTPUT_CHANNELS);
      Slot * end = tmp + OUTPUT_CHANNELS;
      Slot * prev = NULL;
      for(Slot * it = tmp; it != end; ++it)
      {
        if(it->pin == -1) continue;
        if(!prev) it->diff = it->pulse;
        else it->diff = it->pulse - prev->pulse;
        prev = it;
      }
      std::copy(tmp, tmp + OUTPUT_CHANNELS, _slots);
      trigger();
    }

    void trigger() ICACHE_RAM_ATTR;

    Slot* begin() ICACHE_RAM_ATTR
    {
      return _slots;
    }

    Slot* end() ICACHE_RAM_ATTR
    {
      return _slots + OUTPUT_CHANNELS;
    }



  private:
    Slot _buffer[OUTPUT_CHANNELS];
    Slot _slots[OUTPUT_CHANNELS];
    OutputProtocol _protocol;
    uint16_t _rate;
};

extern OutputPWMFast PWMfast;

}

#endif
