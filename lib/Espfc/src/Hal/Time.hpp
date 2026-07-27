#pragma once

#include <cstdint>

extern "C" {

// forward declaration to Arduino functions
#if defined(UNIT_TEST) || defined(ESP8266) || defined(ARCH_RP2040)
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
#else
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
#endif

} // extern "C"
