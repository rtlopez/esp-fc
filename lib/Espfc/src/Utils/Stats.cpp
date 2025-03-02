#include <Arduino.h>
#include "Utils/Stats.h"
#include "Utils/MemoryHelper.h"
#include <cstddef>

namespace Espfc {

namespace Utils {

Stats::Stats(): _loop_last(0), _loop_time(0)
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

void FAST_CODE_ATTR Stats::start(StatCounter c)
{
  _start[c] = micros();
}

void FAST_CODE_ATTR Stats::end(StatCounter c)
{
  uint32_t diff = micros() - _start[c];
  _sum[c] += diff;
  _count[c]++;
}

void Stats::loopTick()
{
  uint32_t now = micros();
  uint32_t diff = now - _loop_last;
  //_loop_time = diff;
  _loop_time += (((int32_t)diff - _loop_time + 4) >> 3);
  _loop_last = now;
}

uint32_t Stats::loopTime() const
{
  return _loop_time;
}

void Stats::update()
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

float Stats::getLoad(StatCounter c) const
{
  return _avg[c] * 100.f;
}

/**
 * @brief Get the Time of counter normalized to 1 ms
 */
float Stats::getTime(StatCounter c) const
{
  return _avg[c] * 1000.0f;
}

float Stats::getReal(StatCounter c) const
{
  return _real[c];
}

float Stats::getFreq(StatCounter c) const
{
  return _freq[c];
}

float Stats::getTotalLoad() const
{
  float ret = 0;
  for(size_t i = 0; i < COUNTER_COUNT; i++)
  {
    if(i == COUNTER_CPU_0 || i == COUNTER_CPU_1) continue;
    ret += getLoad((StatCounter)i);
  }
  return ret;
}

float Stats::getTotalTime() const
{
  float ret = 0;
  for(size_t i = 0; i < COUNTER_COUNT; i++)
  {
    if(i == COUNTER_CPU_0 || i == COUNTER_CPU_1) continue;
    ret += getTime((StatCounter)i);
  }
  return ret;
}

float Stats::getCpuLoad() const
{
  float cpu0 = getLoad(COUNTER_CPU_0);
  float cpu1 = getLoad(COUNTER_CPU_1);
  float maxLoad = std::max(cpu0, cpu1);
  float minLoad = std::min(cpu0, cpu1);
  float alpha = maxLoad / (minLoad + maxLoad);
  return alpha * maxLoad + (1.f - alpha) * minLoad;
}

float Stats::getCpuTime() const
{
  return getTime(COUNTER_CPU_0) + getTime(COUNTER_CPU_1);
}

const char * Stats::getName(StatCounter c) const
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
    case COUNTER_GPS_READ:     return PSTR(" gps_r");
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

}

}
