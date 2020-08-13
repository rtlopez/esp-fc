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
      _bpf.begin(FilterConfig(FILTER_BPF, 180, 80), _rate); // 80 - 180 - 280 [Hz]
      _lpf.begin(FilterConfig(FILTER_BIQUAD, 5), _rate); // 5 Hz
      return 1;
    }

    // calculate noise pitch freq
    void update(float v)
    {
      // pitch detection
      noise = _bpf.update(v);
      bool sign = noise > 0.f;

      // detect rising zero crossing
      if(sign && !_sign_prev) {
        _pitch_freq_raise = Math::clamp((float)_rate / std::max(_pitch_count_raise, 1), _freq_min, _freq_max);
        _pitch_count_raise = 0;
      }

      // detect falling zero crossing
      if(!sign && _sign_prev) {
        _pitch_freq_fall = Math::clamp((float)_rate / std::max(_pitch_count_fall, 1), _freq_min, _freq_max);
        _pitch_count_fall = 0;
      }

      _sign_prev = sign;
      _pitch_count_raise++;
      _pitch_count_fall++;

      freq = lrintf(_lpf.update(std::min(_pitch_freq_raise, _pitch_freq_fall))); // use lower value
    }

    int freq;
    float noise;

  private:
    Filter _bpf;
    Filter _lpf;
    int _rate;

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