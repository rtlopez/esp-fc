#ifndef _ESC_DRIVER_H_
#define _ESC_DRIVER_H_

enum EscProtocol {
  ESC_PROTOCOL_PWM,
  ESC_PROTOCOL_ONESHOT125,
  ESC_PROTOCOL_ONESHOT42,
  ESC_PROTOCOL_MULTISHOT,
  ESC_PROTOCOL_BRUSHED,
  ESC_PROTOCOL_DSHOT150,
  ESC_PROTOCOL_DSHOT300,
  ESC_PROTOCOL_DSHOT600,
  ESC_PROTOCOL_DSHOT1200,
  ESC_PROTOCOL_COUNT
};

#if defined(ESP8266)

  // doesn't support digital
  //#define ESC_PROTOCOL_SANITIZE(p) ((p > ESC_PROTOCOL_BRUSHED || p > ESC_PROTOCOL_MULTISHOT) ? ESC_PROTOCOL_PWM : p)
  #define ESC_PROTOCOL_SANITIZE(p) ((p > ESC_PROTOCOL_BRUSHED) ? ESC_PROTOCOL_PWM : p)

  #define ESC_CHANNEL_COUNT 4
  #include "Esp8266EscDriver.h"
  #define EscDriver Esp8266EscDriver

#elif defined(ESP32)

  // supports analog only for now
  #define ESC_PROTOCOL_SANITIZE(p) (p > ESC_PROTOCOL_BRUSHED ? ESC_PROTOCOL_PWM : p)

  #define ESC_CHANNEL_COUNT RMT_CHANNEL_MAX
  #include "Esp32EscDriver.h"
  #define EscDriver Esp32EscDriver

#else

  #error "Unsupported platform"

#endif

#endif
