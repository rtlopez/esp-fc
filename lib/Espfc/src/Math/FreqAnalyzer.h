#ifndef _ESPFC_MATH_FREQ_ANALYZER_H_
#define _ESPFC_MATH_FREQ_ANALYZER_H_

#include <Arduino.h>
#include "Filter.h"

namespace Espfc {

namespace Math {

class FreqAnalyzer
{
  public:
    FreqAnalyzer() {}

    int begin(int rate)
    {
      _rate = rate;
      _bpf.begin(FilterConfig(FILTER_BPF, 250, 125), _rate); // 125 - 250 - 375 [Hz]
      _lpf.begin(FilterConfig(FILTER_BIQUAD, 3), _rate);
      return 1;
    }

    // return noise pitch freq
    float update(float v)
    {
      // pitch detection
      _noise = _bpf.update(v);
      bool sign = _noise > 0.f;
      if(sign != _sign_prev) {
        _pitch_freq = (float)constrain(_rate / (std::max(_pitch_count, 1) * 2.f), 125, 425);
        _pitch_count = 0;
      }
      _sign_prev = sign;
      _pitch_count++;

      return _lpf.update(_pitch_freq);
    }
  
    Filter _bpf;
    Filter _lpf;
    int _rate;
    float _noise;
    int _pitch_count;
    float _pitch_freq;
    bool _sign_prev;
};

}

}

#endif