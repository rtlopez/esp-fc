#ifndef _ESC_DRIVER_H_
#define _ESC_DRIVER_H_

enum EscProtocol {
  ESC_PROTOCOL_PWM,
  ESC_PROTOCOL_ONESHOT125,
#if defined(ESP32)
  ESC_PROTOCOL_ONESHOT42,
  ESC_PROTOCOL_MULTISHOT,
  ESC_PROTOCOL_DSHOT150,
  ESC_PROTOCOL_DSHOT300,
  ESC_PROTOCOL_DSHOT600,
  ESC_PROTOCOL_DSHOT1200,
#endif
  ESC_PROTOCOL_COUNT
};

#if defined(ESP8266)

  #define ESC_CHANNEL_COUNT 4
  #include "Esp8266EscDriver.h"
  #define EscDriver Esp8266EscDriver

#elif defined(ESP32)

  #define ESC_CHANNEL_COUNT 4
  #include "Esp32EscDriver.h"
  #define EscDriver Esp32EscDriver

#else

  #error "Unsupported platform"

#endif

#endif
