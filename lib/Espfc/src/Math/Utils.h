#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <algorithm>

namespace Espfc {

namespace Math {

class Peak
{
public:
  Peak(): freq(0), value(0) {}
  Peak(float f, float v): freq(f), value(v) {}
  float freq;
  float value;
};

  inline long mapi(long x, long in_min, long in_max, long out_min, long out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  inline float map(float x, float in_min, float in_max, float out_min, float out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  inline float map3(float x, float in_min, float in_neutral, float in_max, float out_min, float out_neutral, float out_max)
  {
    return (x < in_neutral)
      ? map(x, in_min, in_neutral, out_min, out_neutral)
      : map(x, in_neutral, in_max, out_neutral, out_max)
    ;
  }

  template<typename T>
  T deadband(const T value, const T band)
  {
    if(value > band) return value - band;
    else if(value < -band) return value + band;
    return 0;
  }

  // std::clamp() is available since c++17
  template<typename T>
  T clamp(const T value, const T min, const T max)
  {
    if(value > max) return max;
    if(value < min) return min;
    return value;
  }

  inline int alignToClock(uint32_t clock, uint32_t maxFreq)
  {
    uint32_t result = clock;
    uint32_t div = 1;
    while(result > maxFreq)
    {
      result = clock / ++div;
    }
    return result;
  }

  inline uint32_t alignAddressToWrite(uint32_t addr, size_t size, size_t alignment)
  {
    return ((addr + size) / alignment) * alignment;
  }

  constexpr float pi()
  {
    return 3.14159265358979f;
  }

  constexpr float invPi()
  {
    return 1.0f / pi();
  }

  constexpr float inv180()
  {
    return 1.0f / 180.0f;
  }

  inline float toRad(float deg)
  {
    return deg * (pi() * inv180());
  }

  inline float toDeg(float rad)
  {
    return rad * (180.0f * invPi());
  }

  inline float toAltitude(float pressure, float seaLevelPressure = 101325.f)
  {
    return 44330.f * (1.f - std::pow(pressure / seaLevelPressure, 0.1903f));
  }

  inline void peakDetect(float * samples, size_t begin_bin, size_t end_bin, float bin_width, Peak * peaks, size_t peak_count)
  {
    for(size_t b = begin_bin; b <= end_bin; b++)
    {
      if(samples[b] > samples[b - 1] && samples[b] > samples[b + 1])
      {
        float f0 = b * bin_width;
        float k0 = samples[b];

        float fl = f0 - bin_width;
        float kl = samples[b - 1];

        float fh = f0 + bin_width;
        float kh = samples[b + 1];

        // weighted average
        float centerFreq = (k0 * f0 + kl * fl + kh * fh) / (k0 + kl + kh);

        for(size_t p = 0; p < peak_count; p++)
        {
          if(samples[b] > peaks[p].value)
          {
            for(size_t k = (peak_count - 1); k > p; k--)
            {
              peaks[k] = peaks[k - 1];
            }
            peaks[p] = Peak(centerFreq, samples[b]);
            break;
          }
        }
        b++; // next bin can't be peak
      }
    }
  }

  // sort peaks by freq, move zero to end
  void inline peakSort(Peak * peaks, size_t peak_count)
  {
     std::sort(peaks, peaks + peak_count, [](const Peak& a, const Peak& b) -> bool {
      if (a.freq == 0.f) return false;
      if (b.freq == 0.f) return true;
      return a.freq < b.freq;
    });
  }
}

}
