#ifndef _ESPFC_STATS_H_
#define _ESPFC_STATS_H_

#include "Arduino.h"

namespace Espfc {

enum StatCounter {
  COUNTER_IMU_READ,
  COUNTER_IMU_FILTER,
  COUNTER_IMU_FUSION,
  COUNTER_INPUT,
  COUNTER_ACTUATOR,
  COUNTER_OUTER_PID,
  COUNTER_INNER_PID,
  COUNTER_MIXER,
  COUNTER_BLACKBOX,
  COUNTER_TELEMETRY,
  COUNTER_COUNT
};

class Stats
{
  public:
    Stats()
    {
      for(size_t i = 0; i < COUNTER_COUNT; i++) _counter[i] = 0;
    }

    void start(StatCounter c)
    {
      _start[c] = micros();
    }

    void end(StatCounter c)
    {
      uint32_t diff = micros() - _start[c];
      _counter[c] += diff;

      const float k = 0.2f;
      _avg[c] = (k * diff) + ((1.f - k) * _avg[c]);
      //_avg[c] = _avg[c] + ((int32_t)diff - _avg[c]) / 10;
    }

    float getLoad(StatCounter c) const
    {
      return (_counter[c] / 10.0f) / millis();
    }

    float getTime(StatCounter c) const
    {
      return _avg[c];
    }

    float getTotalLoad() const
    {
      float ret = 0;
      for(size_t i = 0; i < COUNTER_COUNT; i++) ret += getLoad((StatCounter)i);
      return ret;
    }

    float getTotalTime() const
    {
      float ret = 0;
      for(size_t i = 0; i < COUNTER_COUNT; i++) ret += getTime((StatCounter)i);
      return ret;
    }

    const char * getName(StatCounter c) const
    {
      switch(c)
      {
        case COUNTER_IMU_READ:
          return PSTR("  imu read");
        case COUNTER_IMU_FILTER:
          return PSTR("imu filter");
        case COUNTER_IMU_FUSION:
          return PSTR("imu fusion");
        case COUNTER_INPUT:
          return PSTR("     input");
        case COUNTER_ACTUATOR:
          return PSTR("  actuator");
        case COUNTER_OUTER_PID:
          return PSTR(" outer pid");
        case COUNTER_INNER_PID:
          return PSTR(" inner pid");
        case COUNTER_MIXER:
          return PSTR("     mixer");
        case COUNTER_BLACKBOX:
          return PSTR("  blackbox");
        case COUNTER_TELEMETRY:
          return PSTR(" telemetry");
        default:
          return PSTR("   unknown");
      }
    }

  private:
    uint32_t _start[COUNTER_COUNT];
    uint64_t _counter[COUNTER_COUNT];
    float _avg[COUNTER_COUNT];
};

}

#endif
