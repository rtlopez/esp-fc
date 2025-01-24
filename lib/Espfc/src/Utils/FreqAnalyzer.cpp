#include "Utils/FreqAnalyzer.hpp"
#include "Utils/Math.hpp"

namespace Espfc {

namespace Utils {

FreqAnalyzer::FreqAnalyzer() {}

int FreqAnalyzer::begin(int rate, DynamicFilterConfig config)
{
  _rate = rate;
  _freq_min = config.min_freq;
  _freq_max = config.max_freq;
  _pitch_freq_raise = _pitch_freq_fall = (_freq_min + _freq_max) * 0.5f;
  _pitch_count_raise = _pitch_count_fall = 0;
  _bpf.begin(FilterConfig(FILTER_BPF, 180, 100), _rate); // 100 - 180 - 260 [Hz]
  return 1;
}

// calculate noise pitch freq
void FreqAnalyzer::update(float v)
{
  // pitch detection
  noise = _bpf.update(v);
  bool sign = noise > 0.f;
  float k = 0.33f;

  // detect rising zero crossing
  if(sign && !_sign_prev) {
    float f = Utils::clamp(_rate / std::max(_pitch_count_raise, 1), _freq_min, _freq_max);
    _pitch_freq_raise += k * (f - _pitch_freq_raise);
    _pitch_count_raise = 0;
  }

  // detect falling zero crossing
  if(!sign && _sign_prev) {
    float f = Utils::clamp(_rate / std::max(_pitch_count_fall, 1), _freq_min, _freq_max);
    _pitch_freq_fall += k * (f - _pitch_freq_fall);
    _pitch_count_fall = 0;
  }

  _sign_prev = sign;
  _pitch_count_raise++;
  _pitch_count_fall++;

  freq = (_pitch_freq_raise + _pitch_freq_fall) * 0.5f;
}

}

}
