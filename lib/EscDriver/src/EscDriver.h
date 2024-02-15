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
  ESC_PROTOCOL_PROSHOT,
  ESC_PROTOCOL_DISABLED,
  ESC_PROTOCOL_COUNT
};

struct EscConfig
{
  int timer;
  EscProtocol protocol;
  int rate;
  bool async;
  bool dshotTelemetry;
};

#define PWM_TO_DSHOT(v) (((v - 1000) * 2) + 47)
#define ESC_PROTOCOL_SANITIZE(p) (p > ESC_PROTOCOL_DSHOT600 && p != ESC_PROTOCOL_DISABLED ? ESC_PROTOCOL_DSHOT600 : p)

class EscDriverBase
{
  public:
#if defined(UNIT_TEST)
    int begin(const EscConfig& conf) { return 1; }
    void end() {}
    int attach(size_t channel, int pin, int pulse) { return 1; }
    int write(size_t channel, int pulse) { return 1; }
    void apply() {}
    int pin(size_t channel) const { return -1; }
#endif

    static inline uint16_t dshotEncode(uint16_t value, bool inverted = false)
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
      if(inverted)
      {
        csum = ~csum;
      }
      csum &= 0xf;

      return (value << 4) | csum;
    }
    
    static const size_t DSHOT_BIT_COUNT = 16;
};

#if defined(ESP8266)

  #define ESC_CHANNEL_COUNT 4
  #include "EscDriverEsp8266.h"
  #define EscDriver EscDriverEsp8266

  #define ESC_DRIVER_MOTOR_TIMER ESC_DRIVER_TIMER1
  #define ESC_DRIVER_SERVO_TIMER ESC_DRIVER_TIMER2

#elif defined(ESP32C3)

  #define ESC_CHANNEL_COUNT 4
  #include "EscDriverEsp32c3.h"
  #define EscDriver EscDriverEsp32c3

  #define ESC_DRIVER_MOTOR_TIMER ESC_DRIVER_TIMER0
  #define ESC_DRIVER_SERVO_TIMER ESC_DRIVER_TIMER1

#elif defined(ESP32S3) || defined(ESP32S2)

  #define ESC_CHANNEL_COUNT 4
  #include "EscDriverEsp32.h"
  #define EscDriver EscDriverEsp32

  #define ESC_DRIVER_MOTOR_TIMER 0
  #define ESC_DRIVER_SERVO_TIMER 1

#elif defined(ESP32)

  #define ESC_CHANNEL_COUNT RMT_CHANNEL_MAX
  #include "EscDriverEsp32.h"
  #define EscDriver EscDriverEsp32

  #define ESC_DRIVER_MOTOR_TIMER 0
  #define ESC_DRIVER_SERVO_TIMER 1

#elif defined(ARCH_RP2040)

  #define ESC_CHANNEL_COUNT 8
  #include "EscDriverRP2040.h"
  #define EscDriver EscDriverRP2040

  #define ESC_DRIVER_MOTOR_TIMER ESC_DRIVER_TIMER0
  #define ESC_DRIVER_SERVO_TIMER ESC_DRIVER_TIMER1

#elif defined(UNIT_TEST)

  #define ESC_CHANNEL_COUNT 4
  #define EscDriver EscDriverBase

  #define ESC_DRIVER_MOTOR_TIMER 0
  #define ESC_DRIVER_SERVO_TIMER 1

#else

  #error "Unsupported platform"

#endif

#endif
