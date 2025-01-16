#pragma once

#include "Utils/Filter.h"

namespace Espfc {

namespace Utils {

class FreqAnalyzer
{
  public:
    FreqAnalyzer();

    int begin(int rate, DynamicFilterConfig config);
    void update(float v);

    float freq;
    float noise;

  private:
    Utils::Filter _bpf;
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
