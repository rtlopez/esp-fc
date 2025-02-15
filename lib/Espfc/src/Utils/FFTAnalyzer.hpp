#pragma once

// https://github.com/espressif/esp-dsp/blob/5f2bfe1f3ee7c9b024350557445b32baf6407a08/examples/fft4real/main/dsps_fft4real_main.c

#include <cstdint>
#include <cstddef>
#include "Utils/Filter.h"
#include "Utils/Math.hpp"

namespace Espfc {

namespace Utils {

enum FFTPhase {
  PHASE_COLLECT,
  PHASE_FFT,
  PHASE_PEAKS
};

template<size_t SAMPLES>
class FFTAnalyzer
{
public:
  FFTAnalyzer();
  ~FFTAnalyzer();

  int begin(int16_t rate, const DynamicFilterConfig& config, size_t axis);
  int update(float v);

  static constexpr size_t PEAKS_MAX = 8;
  Utils::Peak peaks[PEAKS_MAX];

private:
  void clearPeaks();

  static constexpr size_t BINS = SAMPLES >> 1;

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

  // float* _in;
  // float* _out;
  // float* _win;
};

}

}
