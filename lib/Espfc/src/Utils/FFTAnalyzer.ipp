#pragma once

#ifdef ESPFC_DSP

// https://github.com/espressif/esp-dsp/blob/5f2bfe1f3ee7c9b024350557445b32baf6407a08/examples/fft4real/main/dsps_fft4real_main.c
#include "Utils/FFTAnalyzer.hpp"
#include "dsps_fft4r.h"
#include "dsps_wind_hann.h"
#include "esp_heap_caps.h"
#include <algorithm>

namespace Espfc {

namespace Utils {

template<size_t SAMPLES>
FFTAnalyzer<SAMPLES>::FFTAnalyzer(): _idx(0), _phase(PHASE_COLLECT), _begin(0), _end(0), _in(nullptr), _out(nullptr), _win(nullptr)
{
}

template<size_t SAMPLES>
FFTAnalyzer<SAMPLES>::~FFTAnalyzer()
{
  if(_in)  heap_caps_free(_in);
  if(_out) heap_caps_free(_out);
  if(_win) heap_caps_free(_win);
}

template<size_t SAMPLES>
int FFTAnalyzer<SAMPLES>::begin(int16_t rate, const DynamicFilterConfig& config, size_t axis)
{
  if(!_in)  _in  = static_cast<float*>(heap_caps_aligned_alloc(16u, SAMPLES * sizeof(float), MALLOC_CAP_DEFAULT));
  if(!_out) _out = static_cast<float*>(heap_caps_aligned_alloc(16u, SAMPLES * sizeof(float), MALLOC_CAP_DEFAULT));
  if(!_win) _win = static_cast<float*>(heap_caps_aligned_alloc(16u, SAMPLES * sizeof(float), MALLOC_CAP_DEFAULT));

  int16_t nyquistLimit = rate / 2;
  _rate = rate;
  _freq_min = config.min_freq;
  _freq_max = std::min(config.max_freq, nyquistLimit);
  _peak_count = std::min((size_t)config.count, PEAKS_MAX);

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

template<size_t SAMPLES>
// calculate fft and find noise peaks
int FFTAnalyzer<SAMPLES>::update(float v)
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

      Utils::peakDetect(_out, _begin, _end, _bin_width, peaks, _peak_count);

      // sort peaks by freq
      Utils::peakSort(peaks, _peak_count);

      _phase = PHASE_COLLECT;
      return 1;

    default:
      _phase = PHASE_COLLECT;
      return 0;
  }
}

template<size_t SAMPLES>
void FFTAnalyzer<SAMPLES>::clearPeaks()
{
  for(size_t i = 0; i < PEAKS_MAX; i++) peaks[i] = Utils::Peak();
  //std::fill(peaks, peaks + PEAKS_MAX, Peak());
}

}

}

#endif
