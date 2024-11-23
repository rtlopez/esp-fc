#pragma once

#include <cstdint>

#if defined(ARCH_RP2040)
  typedef PinStatus pin_status_t;
  typedef PinMode pin_mode_t;
#else
  typedef uint8_t pin_status_t;
  typedef uint8_t pin_mode_t;
#endif

namespace Espfc {

namespace Hal {

class Gpio
{
public:
  static void digitalWrite(uint8_t pin, pin_status_t val);
  static pin_status_t digitalRead(uint8_t pin);
  static void pinMode(uint8_t pin, pin_mode_t mode);
};

}

}
