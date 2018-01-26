#ifndef _ESPFC_STATS_H_
#define _ESPFC_STATS_H_

#include "Arduino.h"
#include "Timer.h"

namespace Espfc {

enum StatCounter {
  COUNTER_GYRO_READ,
  COUNTER_GYRO_FILTER,
  COUNTER_ACCEL_READ,
  COUNTER_ACCEL_FILTER,
  COUNTER_MAG_READ,
  COUNTER_MAG_FILTER,
  COUNTER_IMU_FUSION,
  COUNTER_IMU_FUSION2,
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
      for(size_t i = 0; i < COUNTER_COUNT; i++)
      {
        _start[i] = 0;
        _avg[i] = 0;
        _sum[i] = 0;
      }
    }

    void start(StatCounter c)
    {
      _start[c] = micros();
    }

    void end(StatCounter c)
    {
      uint32_t diff = micros() - _start[c];
      _sum[c] += diff;
      //const float k = 0.05f;
      //_avg[c] = (k * diff) + ((1.f - k) * _avg[c]);
      //_avg[c] = _avg[c] + ((int32_t)diff - _avg[c]) / 10;
    }

    void calculate()
    {
      for(size_t i = 0; i < COUNTER_COUNT; i++)
      {
        _avg[i] = (float)_sum[i] / timer.deltaUs;
        _sum[i] = 0;
      }
    }

    float getLoad(StatCounter c) const
    {
      return _avg[c] * 100.f;
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
        case COUNTER_GYRO_READ:    return PSTR("   gyro read");
        case COUNTER_GYRO_FILTER:  return PSTR(" gyro filter");
        case COUNTER_ACCEL_READ:   return PSTR("  accel read");
        case COUNTER_ACCEL_FILTER: return PSTR("accel filter");
        case COUNTER_MAG_READ:     return PSTR("    mag read");
        case COUNTER_MAG_FILTER:   return PSTR("  mag filter");
        case COUNTER_IMU_FUSION:   return PSTR(" imu fusion1");
        case COUNTER_IMU_FUSION2:  return PSTR(" imu fusion2");
        case COUNTER_INPUT:        return PSTR("    input rx");
        case COUNTER_ACTUATOR:     return PSTR("    actuator");
        case COUNTER_OUTER_PID:    return PSTR("   pid outer");
        case COUNTER_INNER_PID:    return PSTR("   pid inner");
        case COUNTER_MIXER:        return PSTR("       mixer");
        case COUNTER_BLACKBOX:     return PSTR("    blackbox");
        case COUNTER_TELEMETRY:    return PSTR("   telemetry");
        default:                   return PSTR("     unknown");
      }
    }

    Timer timer;

  private:
    uint32_t _start[COUNTER_COUNT];
    uint32_t _sum[COUNTER_COUNT];
    float _avg[COUNTER_COUNT];
};

}

#endif
