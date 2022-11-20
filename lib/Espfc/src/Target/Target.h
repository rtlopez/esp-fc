#pragma once

#if defined(ESP32)
  #include "TargetESP32.h"
#elif defined(ESP8266)
  #include "TargetESP8266.h"
#elif defined(ARCH_RP2040)
  #include "TargetRP2040.h"
#elif defined(ARCH_NRF52840)
  #include "TargetNRF52840.h"
#elif defined(UNIT_TEST)
  #include "TargetUnitTest.h"
#else
  #error "Unsupported platform!"
#endif