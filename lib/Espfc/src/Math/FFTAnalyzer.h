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

template<size_t SAMPLES>
class FFTAnalyzer
{
public:
  FFTAnalyzer(): _idx(0) {}

  int begin(int16_t rate, const DynamicFilterConfig& config)
  {
    _rate = rate;
    _freq_min = config.min_freq;
    _freq_max = config.max_freq;
    _peak_count = std::min((size_t)config.width, PEAKS_MAX);

    freq = (_freq_min + _freq_max) * 0.5f;

    _idx = 0;
    _bin_width = (float)_rate / SAMPLES; // no need to dived by 2 as we next process `SAMPLES / 2` results
    _bin_offset = _bin_width * 0.5f; // center of bin

    dsps_fft4r_init_fc32(NULL, BINS);

    // Generate hann window
    dsps_wind_hann_f32(_wind, SAMPLES);

    clearPeaks();

    return 1;
  }

  // calculate fft and find noise peaks
  int update(float v)
  {
    _samples[_idx] = v;

    if(++_idx < SAMPLES) return 0; // not enough samples

    _idx = 0;

    // apply window function
    for (size_t j = 0; j < SAMPLES; j++)
    {
      _samples[j] *= _wind[j]; // real
    }

    // FFT Radix-4
    dsps_fft4r_fc32(_samples, BINS);

    // Bit reverse 
    dsps_bit_rev4r_fc32(_samples, BINS);

    // Convert one complex vector with length SAMPLES/2 to one real spectrum vector with length SAMPLES/2
    dsps_cplx2real_fc32(_samples, BINS);

    // calculate magnitude
    for (size_t j = 0; j < BINS; j++)
    {
      size_t k = j * 2;
      _samples[j] = _samples[k] * _samples[k] + _samples[k + 1] * _samples[k + 1];
      //_samples[j] = sqrt(_samples[j]);
    }

    clearPeaks();
    const size_t begin = std::max((size_t)1, (size_t)(_freq_min / _bin_width));
    const size_t end = std::min(BINS - 1, (size_t)(_freq_max / _bin_width));

    for(size_t b = begin; b <= end; b++)
    {
      if(!(_samples[b] > _samples[b - 1] && _samples[b] > _samples[b + 1])) continue;

      float f0 = b * _bin_width;
      float k0 = _samples[b];

      float fl = f0 - _bin_width;
      float kl = _samples[b - 1];

      float fh = f0 + _bin_width;
      float kh = _samples[b + 1];

      // weighted average
      float centerFreq = (k0 * f0 + kl * fl + kh * fh) / (k0 + kl + kh);

      for(size_t p = 0; p < _peak_count; p++)
      {
        if(!(_samples[b] > peaks[p].value)) continue;
        for(size_t k = _peak_count - 1; k > p; k--)
        {
          peaks[k] = peaks[k - 1];
        }
        peaks[p] = Peak(centerFreq, _samples[b]);
      }
      b++; // next bin can't be peak
    }

    // max peak freq
    freq = peaks[0].freq;

    // sort peaks by freq
    std::sort(peaks, peaks + _peak_count, [](const Peak& a, const Peak& b) -> bool {
      if (a.freq == 0.f) return false;
      if (b.freq == 0.f) return true;
      return a.freq > b.freq;
    });
  
    return 1;
  }
  
  float freq;

  static const size_t PEAKS_MAX = 8;
  Peak peaks[PEAKS_MAX];

private:
  void clearPeaks()
  {
    for(size_t i = 0; i < PEAKS_MAX; i++) peaks[i] = Peak();
  }

  static const size_t BINS = SAMPLES >> 1;

  int16_t _rate;
  int16_t _freq_min;
  int16_t _freq_max;
  int16_t _peak_count;

  size_t _idx;
  float _bin_width;
  float _bin_offset;

  // fft input and output
  __attribute__((aligned(16))) float _samples[SAMPLES];
  // Window coefficients
  __attribute__((aligned(16))) float _wind[SAMPLES];
};

}

}

#endif
