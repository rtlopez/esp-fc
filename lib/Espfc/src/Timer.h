#pragma once

#include <cstdint>

namespace Espfc {

class Timer
{
  public:
    Timer();
    int setInterval(uint32_t interval);
    int setRate(uint32_t rate, uint32_t denom = 1u);

    bool check();
    int update();
    bool check(uint32_t now);
    int update(uint32_t now);
    bool syncTo(const Timer& t, uint32_t slot = 0u);

    uint32_t interval;
    uint32_t rate;
    uint32_t denom;

    uint32_t last;
    uint32_t next;
    volatile uint32_t iteration;
    uint32_t delta;
    float intervalf;
};

}
