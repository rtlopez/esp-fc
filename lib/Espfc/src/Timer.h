#ifndef _ESPFC_TIMER_H_
#define _ESPFC_TIMER_H_

#include "Arduino.h"
#include <cstdint>

class Timer
{
  public:
    Timer(): interval(0), last(0), iteration(0), deltaUs(0) {}

    int setInterval(uint32_t interval)
    {
      this->interval = interval;
      this->rate = 1000000UL / interval;
      this->denom = 1;
      iteration = 0;
      deltaUs = 0;
      return 1;
    }

    int setRate(uint32_t rate)
    {
      return setRate(rate, 1);
    }

    int setRate(uint32_t rate, uint32_t denom)
    {
      this->rate = rate / denom;
      this->interval = 1000000UL / this->rate;
      this->denom = denom;
      iteration = 0;
      deltaUs = 0;
      return 1;
    }

    bool check()
    {
      return check(micros());
    }

    int update()
    {
      return update(micros());
    }

    bool check(uint32_t now)
    {
      if(interval == 0) return false;
      if(last + interval > now) return false;
      update(now);
      return true;
    }

    int update(uint32_t now)
    {
      deltaUs = now - last;
      iteration++;
      last = now;
      return 1;
    }

    /*bool sync(Timer& t) const
    {
      if(iteration % t.denom != 0) return false;
      t.update();
      return true;
    }*/

    bool syncTo(const Timer& t)
    {
      if(t.iteration % denom != 0) return false;
      update();
      return true;
    }

    float getDelta() const
    {
      return deltaUs * 0.000001f;
    }

    uint32_t interval;
    uint32_t rate;
    uint32_t denom;

    uint32_t last;
    uint32_t iteration;
    uint32_t deltaUs;
};

#endif
