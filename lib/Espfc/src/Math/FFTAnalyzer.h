#ifndef _ESPFC_MATH_FFT_ANALYZER_H_
#define _ESPFC_MATH_FFT_ANALYZER_H_

// https://github.com/espressif/esp-dsp/blob/5f2bfe1f3ee7c9b024350557445b32baf6407a08/examples/fft4real/main/dsps_fft4real_main.c

#include "Math/Utils.h"
#include "Filter.h"
#include "dsps_fft4r.h"
#include "dsps_wind_hann.h"
#include <algorithm>

namespace Espfc {

namespace Math {

enum FFTPhase {
  PHASE_COLLECT,
  PHASE_FFT,
  PHASE_PEAKS
};

template<size_t SAMPLES>
class FFTAnalyzer
{
public:
  FFTAnalyzer(): _idx(0), _phase(PHASE_COLLECT) {}

  int begin(int16_t rate, const DynamicFilterConfig& config, size_t axis)
  {
    int16_t nyquistLimit = rate / 2;
    _rate = rate;
    _freq_min = config.min_freq;
    _freq_max = std::min(config.max_freq, nyquistLimit);
    _peak_count = std::min((size_t)config.width, PEAKS_MAX);

    _idx = axis * SAMPLES / 3;
    _bin_width = (float)_rate / SAMPLES; // no need to dived by 2 as we next process `SAMPLES / 2` results

    _begin = (_freq_min / _bin_width) + 1;
    _end = std::min(BINS - 1, (size_t)(_freq_max / _bin_width)) - 1;

    // init fft tables
    dsps_fft4r_init_fc32(nullptr, BINS);

    // Generate hann window
    dsps_wind_hann_f32(_win, SAMPLES);

    clearPeaks();

    for(size_t i = 0; i < SAMPLES; i++) _in[i] = 0;
    //std::fill(_in, _in + SAMPLES, 0);

    return 1;
  }

  // calculate fft and find noise peaks
  int update(float v)
  {
    _in[_idx] = v;

    if(++_idx >= SAMPLES)
    {
      _idx = 0;
      _phase = PHASE_FFT;
    }

    switch(_phase)
    {
      case PHASE_COLLECT:
        return 0;

      case PHASE_FFT: // 32us
        // apply window function
        for (size_t j = 0; j < SAMPLES; j++)
        {
          _out[j] = _in[j] * _win[j]; // real
        }

        // FFT Radix-4
        dsps_fft4r_fc32(_out, BINS);

        // Bit reverse
        dsps_bit_rev4r_fc32(_out, BINS);

        // Convert one complex vector with length SAMPLES/2 to one real spectrum vector with length SAMPLES/2
        dsps_cplx2real_fc32(_out, BINS);

        _phase = PHASE_PEAKS;
        return 0;

      case PHASE_PEAKS: // 12us + 22us sqrt()
        // calculate magnitude
        for (size_t j = 0; j < BINS; j++)
        {
          size_t k = j * 2;
          //_out[j] = _out[k] * _out[k] + _out[k + 1] * _out[k + 1];
          _out[j] = sqrt(_out[k] * _out[k] + _out[k + 1] * _out[k + 1]);
        }

        clearPeaks();

        Math::peakDetect(_out, _begin, _end, _bin_width, peaks, _peak_count);

        // sort peaks by freq
        Math::peakSort(peaks, _peak_count);

        _phase = PHASE_COLLECT;
        return 1;

      default:
        _phase = PHASE_COLLECT;
        return 0;
    }
  }

  static const size_t PEAKS_MAX = 8;
  Peak peaks[PEAKS_MAX];

private:
  void clearPeaks()
  {
    for(size_t i = 0; i < PEAKS_MAX; i++) peaks[i] = Peak();
    //std::fill(peaks, peaks + PEAKS_MAX, Peak());
  }

  static const size_t BINS = SAMPLES >> 1;

  int16_t _rate;
  int16_t _freq_min;
  int16_t _freq_max;
  int16_t _peak_count;

  size_t _idx;
  FFTPhase _phase;
  size_t _begin;
  size_t _end;
  float _bin_width;

  // fft input
  __attribute__((aligned(16))) float _in[SAMPLES];

  // fft output
  __attribute__((aligned(16))) float _out[SAMPLES];

  // Window coefficients
  __attribute__((aligned(16))) float _win[SAMPLES];
};

}

}

#endif
