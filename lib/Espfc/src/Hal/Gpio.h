#pragma once

#include <Arduino.h>
#include <cstdint>

#if defined(ARCH_RP2040)
  using pin_status_t = PinStatus;
  using pin_mode_t = PinMode;
#else
  using pin_status_t = uint8_t;
  using pin_mode_t = uint8_t;
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
