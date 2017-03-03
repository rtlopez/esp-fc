#ifndef _OUTPUT_PWM_H_
#define _OUTPUT_PWM_H_

#include "Model.h"

namespace Espfc {

class OutputPWM
{
  public:
    class Slot
    {
      public:
        Slot(): pin(-1), val(0), pulse(0) {}
        char pin;
        bool val;
        volatile int16_t pulse;
        bool operator<(const Slot& rhs) const { return this->pulse < rhs.pulse; }
    };

    OutputPWM();
    int begin(int rate);
    int update();

    inline unsigned long usToTicks(uint32_t us) const ICACHE_RAM_ATTR
    {
      return (clockCyclesPerMicrosecond() * us);// converts microseconds to tick
    }

    int attach(int slot_id, int pin, int pulse)
    {
      if(slot_id < 0 || slot_id >= OUTPUT_CHANNELS) return 0;
      _slots[slot_id].pin = pin;
      _slots[slot_id].pulse = pulse;
      pinMode(pin, OUTPUT);
      return 1;
    }

    int write(int slot_id, int pulse)
    {
      if(slot_id < 0 || slot_id >= OUTPUT_CHANNELS) return 0;
      _slots[slot_id].pulse = pulse;
      return 1;
    }

    inline Slot * getCurrentSlot() ICACHE_RAM_ATTR
    {
      if(_currentSlot == -1) return NULL;
      return _slots + _currentSlot;
    }

    void inline nextSlot() ICACHE_RAM_ATTR
    {
      while(true)
      {
        _currentSlot++;
        if(_currentSlot == OUTPUT_CHANNELS) break;
        if(_slots[_currentSlot].pin == -1) continue; // ignore disabled pins
        break;
      }
      if(_currentSlot == OUTPUT_CHANNELS) _currentSlot = -1;
    }

    int _currentSlot;
    int _frameLength;
    Slot _slots[OUTPUT_CHANNELS];
};

extern OutputPWM PWM;

}

#endif
