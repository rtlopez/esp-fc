#ifndef _ESPFC_FILTER_H_
#define _ESPFC_FILTER_H_

#include <cmath>
#include <algorithm>

namespace Espfc {

enum FilterType {
  FILTER_NONE,
  FILTER_AVG,
  FILTER_BIQUAD,
  FILTER_PT1
};

enum BiquadFilterType {
  BIQUAD_FILTER_LPF,
  BIQUAD_FILTER_NOTCH,
  BIQUAD_FILTER_BPF
};

struct FilterStateAvg {
  float k;
  float v;
};

struct FilterStateBiquad {
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
};

class Filter
{
  public:
    Filter(): _type(FILTER_NONE) {}

    void begin(FilterType type, int cut_freq, int rate)
    {
      _type = type;
      _rate = rate;
      _cut_freq = std::min(_cut_freq, _rate / 2); // adj cut freq below nyquist rule
      switch(_type)
      {
        case FILTER_AVG:
        case FILTER_PT1:
          initPt1();
          break;
        case FILTER_BIQUAD:
          initBiQuad();
          break;
        case FILTER_NONE:
        default:
        ;
      }
    }

    float update(float v)
    {
      switch(_type)
      {
        case FILTER_AVG:
          return updateAvg(v);
        case FILTER_PT1:
          return updatePt1(v);
        case FILTER_BIQUAD:
          return updateBiquad(v);
        case FILTER_NONE:
        default:
          return v;
      }
    }

  private:
    void initPt1()
    {
      float rc = 1.f / (2.f * M_PI * _cut_freq);
      float dt = 1.f / _rate;
      _state.av.k = dt / (dt + rc);
    }

    float updatePt1(float v)
    {
      _state.av.v = _state.av.v + _state.av.k * (v - _state.av.v);
      return _state.av.v;
    }

    float updateAvg(float v)
    {
      _state.av.v = _state.av.v * (1.f - _state.av.k) + _state.av.k * v;
      return _state.av.v;
    }

    void initBiQuad()
    {
      float q = 1.0f / sqrtf(2.0f);     /* quality factor - butterworth*/
      initBiquadType(BIQUAD_FILTER_LPF, q);
    }

    void initBiquadType(BiquadFilterType filterType, float q)
    {
      const float omega = 2.0f * M_PI * _cut_freq * _rate * 0.000001f;
      const float sn = sinf(omega);
      const float cs = cosf(omega);
      const float alpha = sn / (2.0f * q);

      float b0 = 0, b1 = 0, b2 = 0, a0 = 0, a1 = 0, a2 = 0;

      switch (filterType) {
        case BIQUAD_FILTER_LPF:
            b0 = (1 - cs) * 0.5f;
            b1 = 1 - cs;
            b2 = (1 - cs) * 0.5f;
            a0 = 1 + alpha;
            a1 = -2 * cs;
            a2 = 1 - alpha;
            break;
        case BIQUAD_FILTER_NOTCH:
            b0 =  1;
            b1 = -2 * cs;
            b2 =  1;
            a0 =  1 + alpha;
            a1 = -2 * cs;
            a2 =  1 - alpha;
            break;
        case BIQUAD_FILTER_BPF:
            b0 = alpha;
            b1 = 0;
            b2 = -alpha;
            a0 = 1 + alpha;
            a1 = -2 * cs;
            a2 = 1 - alpha;
            break;
      }

      // precompute the coefficients
      _state.bq.b0 = b0 / a0;
      _state.bq.b1 = b1 / a0;
      _state.bq.b2 = b2 / a0;
      _state.bq.a1 = a1 / a0;
      _state.bq.a2 = a2 / a0;

      // zero initial samples
      _state.bq.x1 = _state.bq.x2 = 0;
      _state.bq.y1 = _state.bq.y2 = 0;
    }

    float updateBiquad(float v)
    {
      const float result = _state.bq.b0 * v + _state.bq.x1;
      _state.bq.x1 = _state.bq.b1 * v - _state.bq.a1 * result + _state.bq.x2;
      _state.bq.x2 = _state.bq.b2 * v - _state.bq.a2 * result;
      return result;
    }

    FilterType _type;
    int _cut_freq;
    int _rate;
    union {
      FilterStateAvg av;
      FilterStateBiquad bq;
    } _state;
};

}

#endif
