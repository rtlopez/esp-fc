#include "Hal/Platform.hpp"

#ifdef ESPFC_HAL_DSP

// https://github.com/espressif/esp-dsp/blob/5f2bfe1f3ee7c9b024350557445b32baf6407a08/examples/fft4real/main/dsps_fft4real_main.c
#include "Utils/FFTAnalyzer.hpp"
#include <algorithm>
#include <cmath>

namespace Espfc::Utils {

FFTAnalyzer::FFTAnalyzer(): _idx(0), _phase(PHASE_COLLECT), _begin(0), _end(0) {}

FFTAnalyzer::~FFTAnalyzer() = default;

int FFTAnalyzer::begin(int16_t rate, const DynamicFilterConfig& config, size_t axis)
{
  if (!_in) _in = Hal::Dsp::allocFloats(SAMPLES);
  if (!_out) _out = Hal::Dsp::allocFloats(SAMPLES);
  if (!_win) _win = Hal::Dsp::allocFloats(SAMPLES);

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
  _dsp.init(SAMPLES);

  // Generate hann window
  _dsp.windowHann(_win.get(), SAMPLES);

  clearPeaks();

  for (size_t i = 0; i < SAMPLES; i++)
  {
    _in[i] = 0;
  }

  return 1;
}

// calculate fft and find noise peaks
int FFTAnalyzer::update(float v)
{
  _in[_idx] = v;

  if (++_idx >= SAMPLES)
  {
    _idx = 0;
    _phase = PHASE_FFT;
  }

  switch (_phase)
  {
    case PHASE_COLLECT:
      return 0;

    case PHASE_FFT: // 32us
      // apply window function
      for (size_t j = 0; j < SAMPLES; j++)
      {
        _out[j] = _in[j] * _win[j]; // real
      }

      // Forward real FFT -> interleaved complex spectrum
      _dsp.realForward(_out.get());

      _phase = PHASE_PEAKS;
      return 0;

    case PHASE_PEAKS: // 12us + 22us sqrt()
      // calculate magnitude
      for (size_t j = 0; j < BINS; j++)
      {
        size_t k = j * 2;
        _out[j] = std::sqrt(_out[k] * _out[k] + _out[k + 1] * _out[k + 1]); // amplitude
        //_out[j] = _out[k] * _out[k] + _out[k + 1] * _out[k + 1]; or power
      }

      clearPeaks();

      Utils::peakDetect(_out.get(), _begin, _end, _bin_width, peaks, _peak_count);

      // sort peaks by freq
      Utils::peakSort(peaks, _peak_count);

      _phase = PHASE_COLLECT;
      return 1;

    default:
      _phase = PHASE_COLLECT;
      return 0;
  }
}

void FFTAnalyzer::clearPeaks()
{
  for (size_t i = 0; i < PEAKS_MAX; i++)
  {
    peaks[i] = {};
  }
}

} // namespace Espfc::Utils

#endif
