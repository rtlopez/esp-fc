#pragma once

#include <cstdint>

namespace Espfc::Hal {

class Gpio
{
public:
  enum PinStatus
  {
    Low = 0,
    High = 1
  };

  enum PinMode
  {
    Input = 0,
    InputPullup = 1,
    Output = 10,
  };

  enum InterruptMode
  {
    Rising = 1,
    Falling = 2,
    Change = 3,
  };
  using InterruptHandler = void (*)(void*);

  static void digitalWrite(uint8_t pin, PinStatus val);
  static PinStatus digitalRead(uint8_t pin);
  static void pinMode(uint8_t pin, PinMode mode);
  static void attachInterrupt(uint8_t pin, InterruptHandler handler, void* arg, InterruptMode mode);
  static void detachInterrupt(uint8_t pin);
};

} // namespace Espfc::Hal
