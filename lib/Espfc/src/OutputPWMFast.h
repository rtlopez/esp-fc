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
    int begin(OutputProtocol protocol, bool async, int16_t rate);
    int update();

    inline uint32_t usToTicks(uint32_t us, bool real = false) const ICACHE_RAM_ATTR
    {
      //uint32_t ticks = microsecondsToClockCycles(us); // timer0
      uint32_t ticks = APB_CLK_FREQ / 1000000L * us; // timer1
      //uint32_t ticks = F_CPU / 1000000L / 2 * us; // timer1

      if(!real)
      {
        switch(_protocol)
        {
          case OUTPUT_ONESHOT125:
            ticks = ticks >> 3;
            break;
          default:
            break;
        }
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
      if(_async) return;
      commit();
      trigger();
    }

    void commit() ICACHE_RAM_ATTR;

    void handle() ICACHE_RAM_ATTR;

    void trigger() ICACHE_RAM_ATTR;

    Slot * begin() ICACHE_RAM_ATTR
    {
      return _slots;
    }

    Slot * end() ICACHE_RAM_ATTR
    {
      return _slots + OUTPUT_CHANNELS;
    }

    uint32_t space() const ICACHE_RAM_ATTR
    {
      return _space;
    }

    bool async() const ICACHE_RAM_ATTR
    {
      return _async;
    }

  private:
    Slot _buffer[OUTPUT_CHANNELS];
    Slot _slots[OUTPUT_CHANNELS];

    OutputProtocol _protocol;
    bool _async;
    int16_t _rate;
    uint32_t _interval;
    uint32_t _space;
    volatile bool _isr_busy;
};

extern OutputPWMFast PWMfast;

}

#endif
