#pragma once

// https://github.com/espressif/esp-dsp/blob/5f2bfe1f3ee7c9b024350557445b32baf6407a08/examples/fft4real/main/dsps_fft4real_main.c

#include "Hal/Dsp/Fft.hpp"
#include "Utils/Filter.h"
#include "Utils/Math.hpp"
#include <cstddef>
#include <cstdint>

namespace Espfc::Utils {

enum FFTPhase
{
  PHASE_COLLECT,
  PHASE_FFT,
  PHASE_PEAKS
};

class FFTAnalyzer
{
public:
  FFTAnalyzer();
  ~FFTAnalyzer();

  FFTAnalyzer(const FFTAnalyzer&) = delete;
  FFTAnalyzer& operator=(const FFTAnalyzer&) = delete;
  FFTAnalyzer(FFTAnalyzer&&) = delete;
  FFTAnalyzer& operator=(FFTAnalyzer&&) = delete;

  int begin(int16_t rate, const DynamicFilterConfig& config, size_t axis);
  int update(float v);

  static constexpr size_t PEAKS_MAX = 8;
  Utils::Peak peaks[PEAKS_MAX];

private:
  void clearPeaks();

  static constexpr size_t SAMPLES = 128;
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

  Hal::Dsp::Fft _dsp;
  Hal::Dsp::FloatBuffer _in;
  Hal::Dsp::FloatBuffer _out;
  Hal::Dsp::FloatBuffer _win;
};

} // namespace Espfc::Utils
