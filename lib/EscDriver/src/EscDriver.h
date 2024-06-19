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
    uint32_t telemetry(size_t channel) const { return 0; }
#endif

    static uint16_t IRAM_ATTR dshotConvert(uint16_t pulse)
    {
      return pulse > 1000 ? PWM_TO_DSHOT(pulse) : 0;
    }

    static uint16_t IRAM_ATTR dshotEncode(uint16_t value, bool inverted = false)
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
    
    static uint32_t IRAM_ATTR durationToBitLen(uint32_t duration, uint32_t len)
    {
      return (duration + (len >> 1)) / len;
    }

    static uint32_t IRAM_ATTR pushBits(uint32_t value, uint32_t bitVal, size_t bitLen)
    {
      while(bitLen--)
      {
        value <<= 1;
        value |= bitVal;
      }
      return value;
    }

    /**
     * @param data expected data layout (bits): duration0(15), level0(1), duration(15), level1(1)
     * @param len number of data items
     * @param bitLen duration of single bit
     * @return uint32_t raw gcr value
     */
    static uint32_t IRAM_ATTR extractTelemetryGcr(uint32_t* data, size_t len, uint32_t bitLen)
    {
      int bitCount = 0;
      uint32_t value = 0;
      for(size_t i = 0; i < len; i++)
      {
        uint32_t duration0 = data[i] & 0x7fff;
        if(!duration0) break;
        uint32_t level0 = (data[i] >> 15) & 0x01;
        uint32_t len0 = durationToBitLen(duration0, bitLen);
        if(len0)
        {
          value = pushBits(value, level0, len0);
          bitCount += len0;
        }

        uint32_t duration1 = (data[i] >> 16) & 0x7fff;
        if(!duration1) break;
        uint32_t level1 = (data[i] >> 31) & 0x01;
        uint32_t len1 = durationToBitLen(duration1, bitLen);
        if(len1)
        {
          value = pushBits(value, level1, len1);
          bitCount += len1;
        }
      }

      // fill missing bits with 1
      if(bitCount < 21)
      {
        value = pushBits(value, 0x1, 21 - bitCount);
      }

      return value;
    }

    static constexpr uint32_t INVALID_TELEMETRY_VALUE = 0xffff;
    static constexpr int SECONDS_PER_MINUTE = 60;
    static constexpr float ERPM_PER_LSB = 100.0f;

    static float IRAM_ATTR getErpmToHzRatio(int poles)
    {
      return ERPM_PER_LSB / SECONDS_PER_MINUTE / (poles / 2.0f);
    }

    static uint32_t IRAM_ATTR convertToErpm(uint32_t value)
    {
      if(!value) return 0;

      if(!value || value == INVALID_TELEMETRY_VALUE)
      {
        return INVALID_TELEMETRY_VALUE;
      }

      // Convert period to erpm * 100
      return (1000000 * 60 / 100 + value / 2) / value;
    }

    static uint32_t IRAM_ATTR convertToValue(uint32_t value)
    {
      // eRPM range
      if(value == 0x0fff)
      {
          return 0;
      }

      // Convert value to 16 bit from the GCR telemetry format (eeem mmmm mmmm)
      return (value & 0x01ff) << ((value & 0xfe00) >> 9);
    }

    static uint32_t IRAM_ATTR gcrToRawValue(uint32_t value)
    {
      value = value ^ (value >> 1); // extract gcr

      constexpr uint32_t iv = 0xffffffff; // invalid code
      // First bit is start bit so discard it.
      value &= 0xfffff;
      static const uint32_t decode[32] = {
        iv, iv, iv, iv, iv, iv, iv, iv, iv, 9, 10, 11, iv, 13, 14, 15,
        iv, iv,  2,  3, iv,  5,  6,  7, iv, 0,  8,  1, iv,  4, 12, iv,
      };

      uint32_t decodedValue = decode[value & 0x1f];
      decodedValue |= decode[(value >>  5) & 0x1f] <<  4;
      decodedValue |= decode[(value >> 10) & 0x1f] <<  8;
      decodedValue |= decode[(value >> 15) & 0x1f] << 12;

      uint32_t csum = decodedValue;
      csum = csum ^ (csum >> 8); // xor bytes
      csum = csum ^ (csum >> 4); // xor nibbles

      if((csum & 0xf) != 0xf || decodedValue > 0xffff)
      {
        return INVALID_TELEMETRY_VALUE;
      }
      value = decodedValue >> 4;

      return value;
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
