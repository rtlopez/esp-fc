#pragma once

#include <cstdint>

namespace Espfc::Hal {

struct RgbColor
{
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

class RgbLed
{
public:
  void begin(int8_t pin);
  void write(const RgbColor& color);

private:
  int8_t _pin = -1;
};

} // namespace Espfc::Hal
