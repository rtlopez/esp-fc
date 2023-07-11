#ifndef _ESPFC_MATH_FFT_ANALYZER_H_
#define _ESPFC_MATH_FFT_ANALYZER_H_

// https://github.com/espressif/esp-dsp/blob/5f2bfe1f3ee7c9b024350557445b32baf6407a08/examples/fft4real/main/dsps_fft4real_main.c

#include <algorithm>
#include "Filter.h"
#include "dsps_fft4r.h"
#include "dsps_wind_hann.h"

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

template<size_t Size = 128>
class FFTAnalyzer
{
public:
  FFTAnalyzer(): _idx(0) {}

  int begin(int16_t rate, const DynamicFilterConfig& config)
  {
    _rate = rate;
    _freq_min = config.min_freq;
    _freq_max = config.max_freq;
    _peak_count = config.width;

    freq = (_freq_min + _freq_max) * 0.5f;

    _idx = 0;
    _bin_width = (float)_rate / Size; // no need to dived by 2 as we next process `Size / 2` results
    _bin_offset = _bin_width * 0.5f; // center of bin

    dsps_fft4r_init_fc32(NULL, Size >> 1);

    // Generate hann window
    dsps_wind_hann_f32(_wind, Size);

    clearPeaks();

    return 1;
  }

  // calculate fft and find noise peaks
  int update(float v)
  {
    _samples[_idx] = v;

    if(++_idx < Size) return 0; // not enough samples

    _idx = 0;

    // apply window function
    for (size_t j = 0; j < Size; j++)
    {
      _samples[j] *= _wind[j]; // real
    }

    // FFT Radix-4
    dsps_fft4r_fc32(_samples, Size >> 1);

    // Bit reverse 
    dsps_bit_rev4r_fc32(_samples, Size >> 1);

    // Convert one complex vector with length Size/2 to one real spectrum vector with length Size/2
    dsps_cplx2real_fc32(_samples, Size >> 1);

    // calculate magnitude squared
    for (size_t j = 0; j < Size >> 1; j++)
    {
      size_t k = j * 2;
      _samples[j] = _samples[k] * _samples[k] + _samples[k + 1] * _samples[k + 1];
      //_samples[j] = sqrt(_samples[j]);
    }

    clearPeaks();
    const size_t begin = std::max((size_t)1, (size_t)(_freq_min / _bin_width));
    const size_t end = std::min(BINS - 1, (size_t)(_freq_max / _bin_width));

    float noise = 0;
    size_t noiseCount = 0;
    float valueMax = 0;
    for(size_t b = begin; b <= end; b++)
    {
      noiseCount++;
      float value = _samples[b];
      if (value > valueMax) valueMax = value;
      noise += value;
    }
    noise -= valueMax;
    noise /= noiseCount;

    size_t peakCount = 0;
    for(size_t b = begin; b <= end; b++)
    {
      if(_samples[b] > noise && _samples[b] > _samples[b - 1] && _samples[b] > _samples[b + 1])
      {
        float f0 = b * _bin_width + _bin_offset;
        float k0 = _samples[b];

        float fl = f0 - _bin_width;
        float kl = _samples[b - 1];

        float fh = f0 + _bin_width;
        float kh = _samples[b + 1];

        // weighted average
        float centerFreq = (k0 * f0 + kl * fl + kh * fh) / (k0 + kl + kh);

        _peaks[peakCount] = Peak(centerFreq, _samples[b]);

        peakCount++;
        b++; // next bin can't be peak
      }
    }

    if(peakCount > 1)
    {
      // sort peaks by value
      std::sort(_peaks, _peaks + peakCount, [](const Peak& a, const Peak& b) -> bool {
        return a.value < b.value;
      });
    }

    freq = _peaks[0].freq;

    return 1;
  }
  
  float freq;

private:
  void clearPeaks()
  {
    for(size_t i = 0; i < PEAKS_COUNT; i++) _peaks[i] = Peak();
  }

  static const size_t BINS = Size >> 1;

  int16_t _rate;
  int16_t _freq_min;
  int16_t _freq_max;
  int16_t _peak_count;

  size_t _idx;
  float _bin_width;
  float _bin_offset;

  static const size_t PEAKS_COUNT = BINS >> 1;
  Peak _peaks[PEAKS_COUNT];

  // fft input and output
  __attribute__((aligned(16))) float _samples[Size];
  // Window coefficients
  __attribute__((aligned(16))) float _wind[Size];
};

}

}

#endif
