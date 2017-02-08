#ifndef _ESPFC_MATH_H_
#define _ESPFC_MATH_H_

namespace Espfc {

class Math
{
public:
  float static map(float x, float in_min, float in_max, float out_min, float out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  float static map3(float x, float in_min, float in_neutral, float in_max, float out_min, float out_neutral, float out_max)
  {
    return (x < in_neutral)
      ? Math::map(x, in_min, in_neutral, out_min, out_neutral)
      : Math::map(x, in_neutral, in_max, out_neutral, out_max)
    ;
  }

  template<typename T>
  T static bound(T x, T min, T max)
  {
    if(x > max) return max;
    if(x < min) return min;
    return x;
  }
};

}

#endif
