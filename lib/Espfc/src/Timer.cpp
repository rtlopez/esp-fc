#include <Arduino.h>
#include "Timer.h"
#include "Utils/MemoryHelper.h"

namespace Espfc {

Timer::Timer(): interval(0), last(0), next(0), iteration(0), delta(0)
{
}

int Timer::setInterval(uint32_t interval)
{
  this->interval = interval;
  this->rate = (1000000UL + interval / 2) / interval;
  this->denom = 1;
  this->delta = this->interval;
  this->intervalf = this->interval * 0.000001f;
  iteration = 0;
  return 1;
}

int Timer::setRate(uint32_t rate, uint32_t denom)
{
  this->rate = (rate + denom / 2) / denom;
  this->interval = (1000000UL + this->rate / 2) / this->rate;
  this->denom = denom;
  this->delta = this->interval;
  this->intervalf = this->interval * 0.000001f;
  iteration = 0;
  return 1;
}

bool FAST_CODE_ATTR Timer::check()
{
  return check(micros());
}

int FAST_CODE_ATTR Timer::update()
{
  return update(micros());
}

bool FAST_CODE_ATTR Timer::check(uint32_t now)
{
  if(interval == 0) return false;
  if(now < next) return false;
  return update(now);
}

int FAST_CODE_ATTR Timer::update(uint32_t now)
{
  next = now + interval;
  delta = now - last;
  last = now;
  iteration++;
  return 1;
}

bool FAST_CODE_ATTR Timer::syncTo(const Timer& t, uint32_t slot)
{
  if(denom > 0)
  {
    if(slot > denom - 1) slot = denom - 1;
    if(t.iteration % denom != slot) return false;
    return update(micros());
  }
  return check(micros());
}

}
