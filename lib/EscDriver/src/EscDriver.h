#ifndef _ESC_DRIVER_H_
#define _ESC_DRIVER_H_

#include <Arduino.h>

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
  ESC_PROTOCOL_PROSHOT,
  ESC_PROTOCOL_COUNT
};

#define PWM_TO_DSHOT(v) (((v - 1000) * 2) + 47)

class EscDriverBase
{
  public:
    uint16_t dshotEncode(uint16_t value)
    {
      value <<= 1;

      // compute checksum
      int csum = 0;
      int csum_data = value;
      for (int i = 0; i < 3; i++)
      {
        csum ^= csum_data; // xor
        csum_data >>= 4;
      }
      csum &= 0xf;

      return (value << 4) | csum;
    }
    
    static const size_t DSHOT_BIT_COUNT = 16;
};

#if defined(ESP8266)

  // supports dshot600 max
  #define ESC_PROTOCOL_SANITIZE(p) (p > ESC_PROTOCOL_DSHOT600 ? ESC_PROTOCOL_DSHOT600 : p)

  #define ESC_CHANNEL_COUNT 4
  #include "EscDriverEsp8266.h"
  #define EscDriver EscDriverEsp8266

  #define ESC_DRIVER_MOTOR_TIMER ESC_DRIVER_TIMER1
  #define ESC_DRIVER_SERVO_TIMER ESC_DRIVER_TIMER2

#elif defined(ESP32)

  // supports analog only for now
  #define ESC_PROTOCOL_SANITIZE(p) (p > ESC_PROTOCOL_DSHOT1200 ? ESC_PROTOCOL_DSHOT1200 : p)

  #define ESC_CHANNEL_COUNT RMT_CHANNEL_MAX
  #include "EscDriverEsp32.h"
  #define EscDriver EscDriverEsp32

  #define ESC_DRIVER_MOTOR_TIMER 0
  #define ESC_DRIVER_SERVO_TIMER 0

#else

  #error "Unsupported platform"

#endif

#endif
