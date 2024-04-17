#include "Timer.h"
#include <Arduino.h>

namespace Espfc {

Timer::Timer(): interval(0), last(0), next(0), iteration(0), delta(0)
{
}

int Timer::setInterval(uint32_t interval)
{
    this->interval = interval;
    this->rate = 1000000UL / interval;
    this->denom = 1;
    this->delta = this->interval;
    this->intervalf = this->interval * 0.000001f;
    iteration = 0;
    return 1;
}

int Timer::setRate(uint32_t rate, uint32_t denom)
{
    this->rate = rate / denom;
    this->interval = 1000000UL / this->rate;
    this->denom = denom;
    this->delta = this->interval;
    this->intervalf = this->interval * 0.000001f;
    iteration = 0;
    return 1;
}

bool Timer::check()
{
    return check(micros());
}

int Timer::update()
{
    return update(micros());
}

bool Timer::check(uint32_t now)
{
    if(interval == 0) return false;
    if(now < next) return false;
    return update(now);
}

int Timer::update(uint32_t now)
{
    next = now + interval;
    delta = now - last;
    last = now;
    iteration++;
    return 1;
}

bool Timer::syncTo(const Timer& t)
{
    if(t.iteration % denom != 0) return false;
    return update();
}

}
