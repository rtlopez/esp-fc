#pragma once

#include "Utils/Timer.h"
#include <cstdint>

namespace Espfc {

enum StatCounter {
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
  COUNTER_GPS_READ,
  COUNTER_CPU_0,
  COUNTER_CPU_1,
  COUNTER_COUNT
};

namespace Utils {

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

    Stats();

    void start(StatCounter c);
    void end(StatCounter c);

    void update();
    
    void loopTick();
    uint32_t loopTime() const;
    float getLoad(StatCounter c) const;

    /**
     * @brief Get the Time of counter normalized to 1 ms
     */
    float getTime(StatCounter c) const;
    float getReal(StatCounter c) const;
    float getFreq(StatCounter c) const;
    float getTotalLoad() const;
    float getTotalTime() const;
    float getCpuLoad() const;
    float getCpuTime() const;
    const char * getName(StatCounter c) const;

    Utils::Timer timer;

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

}
