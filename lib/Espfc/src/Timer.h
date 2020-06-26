#ifndef _ESPFC_TIMER_H_
#define _ESPFC_TIMER_H_

#include <Arduino.h>
#include <cstdint>

namespace Espfc {

class Timer
{
  public:
    Timer(): interval(0), last(0), iteration(0), delta(0) {}

    int setInterval(uint32_t interval)
    {
      this->interval = interval;
      this->rate = 1000000UL / interval;
      this->denom = 1;
      this->delta = this->interval;
      this->intervalf = this->interval * 0.000001f;
      iteration = 0;
      return 1;
    }

    int setRate(uint32_t rate, uint32_t denom = 1)
    {
      this->rate = rate / denom;
      this->interval = 1000000UL / this->rate;
      this->denom = denom;
      this->delta = this->interval;
      this->intervalf = this->interval * 0.000001f;
      iteration = 0;
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
      return update(now);
    }

    int update(uint32_t now)
    {
      delta = now - last;
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
      return update();
    }

    uint32_t interval;
    uint32_t rate;
    uint32_t denom;

    uint32_t last;
    uint32_t iteration;
    uint32_t delta;
    float intervalf;
};

}

#endif
