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

    bool check() IRAM_ATTR_ALT;
    int update() IRAM_ATTR_ALT;
    bool check(uint32_t now) IRAM_ATTR_ALT;
    int update(uint32_t now) IRAM_ATTR_ALT;
    bool syncTo(const Timer& t) IRAM_ATTR_ALT;

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
