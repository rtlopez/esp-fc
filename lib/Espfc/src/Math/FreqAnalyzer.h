#ifndef _ESPFC_MATH_FREQ_ANALYZER_H_
#define _ESPFC_MATH_FREQ_ANALYZER_H_

#include "Math/Utils.h"
#include "Filter.h"

namespace Espfc {

namespace Math {

class FreqAnalyzer
{
  public:
    FreqAnalyzer() {}

    int begin(int rate, DynamicFilterConfig config)
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
    void update(float v)
    {
      // pitch detection
      noise = _bpf.update(v);
      bool sign = noise > 0.f;
      float k = 0.33f;

      // detect rising zero crossing
      if(sign && !_sign_prev) {
        float f = Math::clamp(_rate / std::max(_pitch_count_raise, 1), _freq_min, _freq_max);
        _pitch_freq_raise += k * (f - _pitch_freq_raise);
        _pitch_count_raise = 0;
      }

      // detect falling zero crossing
      if(!sign && _sign_prev) {
        float f = Math::clamp(_rate / std::max(_pitch_count_fall, 1), _freq_min, _freq_max);
        _pitch_freq_fall += k * (f - _pitch_freq_fall);
        _pitch_count_fall = 0;
      }

      _sign_prev = sign;
      _pitch_count_raise++;
      _pitch_count_fall++;

      freq = (_pitch_freq_raise + _pitch_freq_fall) * 0.5f;
    }

    float freq;
    float noise;

  private:
    Filter _bpf;
    float _rate;

    float _freq_min;
    float _freq_max;

    int _pitch_count_raise;
    int _pitch_count_fall;

    float _pitch_freq_raise;
    float _pitch_freq_fall;

    bool _sign_prev;
};

}

}

#endif
