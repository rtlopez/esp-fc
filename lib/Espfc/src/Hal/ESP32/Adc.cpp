#if defined(ESP32)

#include "Hal/Adc.hpp"
#include <Arduino.h>

namespace Espfc::Hal {

void Adc::begin(uint8_t pin)
{
  ::analogReadResolution(12);
}

uint16_t Adc::read(uint8_t pin)
{
  return ::analogRead(pin);
}

} // namespace Espfc::Hal

#endif
