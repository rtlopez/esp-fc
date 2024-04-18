#pragma once

#include <cstdint>
#include "Utils/MemoryHelper.h"

namespace Espfc {

class Timer
{
  public:
    Timer();
    int setInterval(uint32_t interval);
    int setRate(uint32_t rate, uint32_t denom = 1);

    bool check() FAST_CODE_ATTR;
    int update() FAST_CODE_ATTR;
    bool check(uint32_t now) FAST_CODE_ATTR;
    int update(uint32_t now) FAST_CODE_ATTR;
    bool syncTo(const Timer& t) FAST_CODE_ATTR;

    uint32_t interval;
    uint32_t rate;
    uint32_t denom;

    uint32_t last;
    uint32_t next;
    uint32_t iteration;
    uint32_t delta;
    float intervalf;
};

}
