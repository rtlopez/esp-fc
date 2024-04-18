#pragma once

#include "Utils/MemoryHelper.h"
#include <cstdint>

#if defined(ARCH_RP2040)
typedef PinStatus pin_status_t;
#else
typedef uint8_t pin_status_t;
#endif

namespace Espfc {

namespace Hal {

class Gpio
{
  public:
    static void digitalWrite(uint8_t pin, pin_status_t val) FAST_CODE_ATTR;
    static pin_status_t digitalRead(uint8_t pin) FAST_CODE_ATTR;
};

}

}
