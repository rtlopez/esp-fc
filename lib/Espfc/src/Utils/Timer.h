#pragma once

#include <cstdint>

namespace Espfc {

namespace Utils {

class Timer
{
  public:
    Timer() = default;
    int setInterval(uint32_t interval);
    int setRate(uint32_t rate, uint32_t denom = 1u);

    bool check();
    int update();
    bool check(uint32_t now);
    int update(uint32_t now);
    bool syncTo(const Timer& t, uint32_t slot = 0u);

    uint32_t interval = 0;
    uint32_t rate = 0;
    uint32_t denom = 0;

    uint32_t last = 0;
    uint32_t next = 0;
    volatile uint32_t iteration = 0;
    uint32_t delta = 0;
    float intervalf = 0.f;
    float realRate = 0.f;
};

}

}
