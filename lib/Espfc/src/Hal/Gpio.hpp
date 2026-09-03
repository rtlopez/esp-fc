#pragma once

#include <cstdint>

namespace Espfc::Hal {

class Gpio
{
public:
  enum PinStatus {
    Low = 0,
    High = 1
  };
  enum PinMode {
    Input = 0,
    InputPullup = 1,
    Output = 10,
  };

  static void digitalWrite(uint8_t pin, PinStatus val);
  static PinStatus digitalRead(uint8_t pin);
  static void pinMode(uint8_t pin, PinMode mode);
};

} // namespace Espfc::Hal
