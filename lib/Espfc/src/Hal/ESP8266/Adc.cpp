#if defined(ESP8266)

#include "Hal/Adc.hpp"
#include <Arduino.h>

namespace Espfc::Hal {

void Adc::begin(uint8_t pin)
{
  // resolution is fixed to 10 bits
}

uint16_t Adc::read(uint8_t pin)
{
  return ::analogRead(pin);
}

} // namespace Espfc::Hal

#endif
