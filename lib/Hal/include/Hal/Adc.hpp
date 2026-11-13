#pragma once

#include <cstdint>

namespace Hal {

class Adc
{
public:
  static void begin(uint8_t pin);
  static uint16_t read(uint8_t pin);
};

} // namespace Hal
