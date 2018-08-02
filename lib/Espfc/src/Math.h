#ifndef _ESPFC_MATH_H_
#define _ESPFC_MATH_H_

namespace Espfc {

namespace Math {

  static float map(float x, float in_min, float in_max, float out_min, float out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  static float map3(float x, float in_min, float in_neutral, float in_max, float out_min, float out_neutral, float out_max)
  {
    return (x < in_neutral)
      ? Math::map(x, in_min, in_neutral, out_min, out_neutral)
      : Math::map(x, in_neutral, in_max, out_neutral, out_max)
    ;
  }

  template<typename T>
  T deadband(const T value, const T band)
  {
    if(value > band) return value - band;
    else if(value < -band) return value + band;
    return 0;
  }

  template<typename T>
  T bound(const T x, const T min, const T max)
  {
    if(x > max) return max;
    if(x < min) return min;
    return x;
  }

}

}

#endif
