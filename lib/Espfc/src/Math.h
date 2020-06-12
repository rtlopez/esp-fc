#ifndef _ESPFC_MATH_H_
#define _ESPFC_MATH_H_

namespace Espfc {

namespace Math {

  inline float map(float x, float in_min, float in_max, float out_min, float out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  inline float map3(float x, float in_min, float in_neutral, float in_max, float out_min, float out_neutral, float out_max)
  {
    return (x < in_neutral)
      ? map(x, in_min, in_neutral, out_min, out_neutral)
      : map(x, in_neutral, in_max, out_neutral, out_max)
    ;
  }

  template<typename T>
  T deadband(const T value, const T band)
  {
    if(value > band) return value - band;
    else if(value < -band) return value + band;
    return 0;
  }

  // std::clamp() is available since c++17
  template<typename T>
  T clamp(const T value, const T min, const T max)
  {
    if(value > max) return max;
    if(value < min) return min;
    return value;
  }

  constexpr float pi()
  {
    return 3.14159265358979f;
  }
}

}

#endif
