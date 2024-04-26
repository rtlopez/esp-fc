#ifndef _ESPFC_STATS_H_
#define _ESPFC_STATS_H_

#include "Arduino.h"
#include "Timer.h"

namespace Espfc {

enum StatCounter : int8_t {
  COUNTER_GYRO_READ,
  COUNTER_GYRO_FILTER,
  COUNTER_GYRO_FFT,
  COUNTER_RPM_UPDATE,
  COUNTER_ACCEL_READ,
  COUNTER_ACCEL_FILTER,
  COUNTER_MAG_READ,
  COUNTER_MAG_FILTER,
  COUNTER_BARO,
  COUNTER_IMU_FUSION,
  COUNTER_IMU_FUSION2,
  COUNTER_OUTER_PID,
  COUNTER_INNER_PID,
  COUNTER_MIXER,
  COUNTER_MIXER_WRITE,
  COUNTER_MIXER_READ,
  COUNTER_BLACKBOX,
  COUNTER_INPUT_READ,
  COUNTER_INPUT_FILTER,
  COUNTER_FAILSAFE,
  COUNTER_ACTUATOR,
  COUNTER_TELEMETRY,
  COUNTER_SERIAL,
  COUNTER_WIFI,
  COUNTER_BATTERY,
  COUNTER_CPU_0,
  COUNTER_CPU_1,
  COUNTER_COUNT
};

class Stats
{
  public:
    class Measure
    {
      public:
        inline Measure(Stats& stats, StatCounter counter): _stats(stats), _counter(counter)
        {
          _stats.start(_counter);
        }
        inline ~Measure()
        {
          _stats.end(_counter);
        }
      private:
        Stats& _stats;
        StatCounter _counter;
    };

    Stats(): _loop_last(0), _loop_time(0)
    {
      for(size_t i = 0; i < COUNTER_COUNT; i++)
      {
        _start[i] = 0;
        _avg[i] = 0;
        _sum[i] = 0;
        _count[i] = 0;
        _freq[i] = 0;
      }
    }

    inline void start(StatCounter c) IRAM_ATTR
    {
      _start[c] = micros();
    }

    inline void end(StatCounter c) IRAM_ATTR
    {
      uint32_t diff = micros() - _start[c];
      _sum[c] += diff;
      _count[c]++;
    }

    void loopTick()
    {
      uint32_t now = micros();
      uint32_t diff = now - _loop_last;
      //_loop_time = diff;
      _loop_time += (((int32_t)diff - _loop_time + 4) >> 3);
      _loop_last = now;
    }

    uint32_t loopTime() const
    {
      return _loop_time;
    }

    void update()
    {
      if(!timer.check()) return;
      for(size_t i = 0; i < COUNTER_COUNT; i++)
      {
        _avg[i] = (float)(_sum[i] + (_count[i] >> 1)) / timer.delta;
        _freq[i] = (float)_count[i] * 1e6 / timer.delta;
        _real[i] = _count[i] > 0 ? ((float)(_sum[i] + (_count[i] >> 1)) / _count[i]) : 0.0f;
        _sum[i] = 0;
        _count[i] = 0;
      }
    }

    float getLoad(StatCounter c) const
    {
      return _avg[c] * 100.f;
    }

    /**
     * @brief Get the Time of counter normalized to 1 ms
     */
    float getTime(StatCounter c) const
    {
      return _avg[c] * 1000.0f;
    }

    float getReal(StatCounter c) const
    {
      return _real[c];
    }

    float getFreq(StatCounter c) const
    {
      return _freq[c];
    }

    float getTotalLoad() const
    {
      float ret = 0;
      for(size_t i = 0; i < COUNTER_COUNT; i++)
      {
        if(i == COUNTER_CPU_0 || i == COUNTER_CPU_1) continue;
        ret += getLoad((StatCounter)i);
      }
      return ret;
    }

    float getTotalTime() const
    {
      float ret = 0;
      for(size_t i = 0; i < COUNTER_COUNT; i++)
      {
        if(i == COUNTER_CPU_0 || i == COUNTER_CPU_1) continue;
        ret += getTime((StatCounter)i);
      }
      return ret;
    }

    float getCpuLoad() const
    {
      float cpu0 = getLoad(COUNTER_CPU_0);
      float cpu1 = getLoad(COUNTER_CPU_1);
      float maxLoad = std::max(cpu0, cpu1);
      float minLoad = std::min(cpu0, cpu1);
      float alpha = maxLoad / (minLoad + maxLoad);
      return alpha * maxLoad + (1.f - alpha) * minLoad;
    }

    float getCpuTime() const
    {
      return getTime(COUNTER_CPU_0) + getTime(COUNTER_CPU_1);
    }

    const char * getName(StatCounter c) const
    {
      switch(c)
      {
        case COUNTER_GYRO_READ:    return PSTR("gyro_r");
        case COUNTER_GYRO_FILTER:  return PSTR("gyro_f");
        case COUNTER_GYRO_FFT:     return PSTR("gyro_a");
        case COUNTER_RPM_UPDATE:   return PSTR(" rpm_u");
        case COUNTER_ACCEL_READ:   return PSTR(" acc_r");
        case COUNTER_ACCEL_FILTER: return PSTR(" acc_f");
        case COUNTER_MAG_READ:     return PSTR(" mag_r");
        case COUNTER_MAG_FILTER:   return PSTR(" mag_f");
        case COUNTER_BARO:         return PSTR("baro_p");
        case COUNTER_IMU_FUSION:   return PSTR(" imu_p");
        case COUNTER_IMU_FUSION2:  return PSTR(" imu_c");
        case COUNTER_OUTER_PID:    return PSTR(" pid_o");
        case COUNTER_INNER_PID:    return PSTR(" pid_i");
        case COUNTER_MIXER:        return PSTR(" mix_p");
        case COUNTER_MIXER_WRITE:  return PSTR(" mix_w");
        case COUNTER_MIXER_READ:   return PSTR(" mix_r");
        case COUNTER_BLACKBOX:     return PSTR(" bblog");
        case COUNTER_INPUT_READ:   return PSTR("  rx_r");
        case COUNTER_INPUT_FILTER: return PSTR("  rx_f");
        case COUNTER_FAILSAFE:     return PSTR("  rx_s");
        case COUNTER_ACTUATOR:     return PSTR("  rx_a");
        case COUNTER_SERIAL:       return PSTR("serial");
        case COUNTER_WIFI:         return PSTR("  wifi");
        case COUNTER_BATTERY:      return PSTR("   bat");
        case COUNTER_TELEMETRY:    return PSTR("   tlm");
        case COUNTER_CPU_0:        return PSTR(" cpu_0");
        case COUNTER_CPU_1:        return PSTR(" cpu_1");
        default:                   return PSTR("unknwn");
      }
    }

    Timer timer;

  private:
    uint32_t _start[COUNTER_COUNT];
    uint32_t _sum[COUNTER_COUNT];
    uint32_t _count[COUNTER_COUNT];
    float _avg[COUNTER_COUNT];
    float _freq[COUNTER_COUNT];
    float _real[COUNTER_COUNT];
    uint32_t _loop_last;
    int32_t _loop_time;
};

}

#endif
