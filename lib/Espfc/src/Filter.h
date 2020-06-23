#ifndef _ESPFC_FILTER_H_
#define _ESPFC_FILTER_H_

#include "Math/Utils.h"
#include <cmath>

// Quick median filter implementation
// (c) N. Devillard - 1998
// http://ndevilla.free.fr/median/median.pdf
#define QMF_SORT(a,b) { if ((a)>(b)) QMF_SWAP((a),(b)); }
#define QMF_SWAP(a,b) { int32_t temp=(a);(a)=(b);(b)=temp; }
#define QMF_COPY(p,v,n) { for (size_t i=0; i<n; i++) p[i]=v[i]; }
#define QMF_SORTF(a,b) { if ((a)>(b)) QMF_SWAPF((a),(b)); }
#define QMF_SWAPF(a,b) { float temp=(a);(a)=(b);(b)=temp; }

namespace Espfc {

enum FilterType {
  FILTER_PT1,
  FILTER_BIQUAD,
  FILTER_PT1_FIR2,
  FILTER_NOTCH,
  FILTER_FIR2,
  FILTER_MEDIAN3,
  FILTER_BPF,
  FILTER_NOTCH_DF1,
  FILTER_NONE,
};

enum BiquadFilterType {
  BIQUAD_FILTER_LPF,
  BIQUAD_FILTER_NOTCH,
  BIQUAD_FILTER_BPF
};

class FilterConfig
{
  public:
    FilterConfig() {}
    FilterConfig(FilterType t, int16_t f, int16_t c = 0): type(t), freq(f), cutoff(c) {}
    int8_t type;
    int16_t freq;
    int16_t cutoff;
};

struct FilterStatePt1 {
  float k;
  float v;
  float v0;
};

struct FilterStateBiquad {
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
};

struct FilterStateMedian {
  float v[3];
  int i;
};


class Filter
{
  public:
    Filter(): _type(FILTER_NONE) {}

    void begin()
    {
      _type = FILTER_NONE;
    }

    void begin(const FilterConfig& config, int rate)
    {
      _type = (FilterType)config.type;
      _rate = rate;
      _freq = Math::clamp((int)config.freq, 0, _rate / 2); // adj cut freq below nyquist rule
      _cutoff = Math::clamp((int)config.cutoff, 0, _rate / 2); // adj cut freq below nyquist rule
      _cutoff = Math::clamp(_cutoff, 0, _freq - 10); // sanitize cutoff to be slightly below filter freq
      if(_freq == 0) _type = FILTER_NONE; // turn off if filter freq equals zero
      switch(_type)
      {
        case FILTER_PT1:
          initPt1();
          break;
        case FILTER_BIQUAD:
          initBiquadLpf();
          break;
        case FILTER_PT1_FIR2:
          initPt1();
          break;
        case FILTER_NOTCH:
        case FILTER_NOTCH_DF1:
          initBiquadNotch();
          break;
        case FILTER_FIR2:
          initFir2();
          break;
        case FILTER_MEDIAN3:
          initMedian3();
          break;
        case FILTER_BPF:
          initBiquadBpf();
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
        case FILTER_PT1:
          return updatePt1(v);
        case FILTER_BIQUAD:
          return updateBiquadDF2(v);
        case FILTER_PT1_FIR2:
          return updatePt1Fir2(v);
        case FILTER_NOTCH:
          return updateBiquadDF2(v);
        case FILTER_FIR2:
          return updateFir2(v);
        case FILTER_MEDIAN3:
          return updateMedian3(v);
        case FILTER_BPF:
          return updateBiquadDF2(v);
        case FILTER_NOTCH_DF1:
          return updateBiquadDF1(v);
        case FILTER_NONE:
        default:
          return v;
      }
    }

    void reconfigureNotchDF1(float freq, float cutoff)
    {
      _freq = freq;
      _cutoff = cutoff;
      //float q = 1.7f; // or const Q factor
      float q = getNotchQApprox(freq, cutoff);
      //float q = getNotchQ(freq, cutoff);
      initBiquadCoefficients(BIQUAD_FILTER_NOTCH, q);
    }

    float getNotchQApprox(float freq, float cutoff)
    {
      return ((float)(cutoff * freq) / ((float)(freq - cutoff) * (float)(freq + cutoff)));
    }

    float getNotchQ(int freq, int cutoff)
    {
      float octaves = log2f((float)freq  / (float)cutoff) * 2;
      return sqrtf(powf(2, octaves)) / (powf(2, octaves) - 1);
    }

  private:
    void initPt1()
    {
      float rc = 1.f / (2.f * Math::pi() * _freq);
      float dt = 1.f / _rate;
      _state.pt1.k = dt / (dt + rc);
      _state.pt1.v = 0.f;
      _state.pt1.v0 = 0.f;
    }

    void initFir2()
    {
      _state.pt1.v = 0.f;
      _state.pt1.v0 = 0.f;
    }

    void initMedian3()
    {
      _state.median.v[0] = 0.f;
      _state.median.v[1] = 0.f;
      _state.median.v[2] = 0.f;
      _state.median.i = 0;
    }

    float updatePt1(float v) ICACHE_RAM_ATTR
    {
      _state.pt1.v += _state.pt1.k * (v - _state.pt1.v);
      return _state.pt1.v;
    }

    float updatePt1Fir2(float v) /* ICACHE_RAM_ATTR */
    {
      _state.pt1.v += _state.pt1.k * (v - _state.pt1.v);
      const float t = _state.pt1.v;
      _state.pt1.v = (_state.pt1.v + _state.pt1.v0) * 0.5f;
      _state.pt1.v0 = t;
      return _state.pt1.v;
    }

    float updateFir2(float v) /* ICACHE_RAM_ATTR */
    {
      _state.pt1.v = (v + _state.pt1.v0) * 0.5f;
      _state.pt1.v0 = v;
      return _state.pt1.v;
    }

    float updateMedian3(float v)
    {
      _state.median.v[0] = _state.median.v[1];
      _state.median.v[1] = _state.median.v[2];
      _state.median.v[2] = v;
      float p[3];
      QMF_COPY(p, _state.median.v, 3);
      QMF_SORTF(p[0], p[1]); QMF_SORTF(p[1], p[2]); QMF_SORTF(p[0], p[1]);
      return p[1];
    }

    void initBiquadNotch()
    {
      float q = getNotchQ(_freq, _cutoff);
      initBiquad(BIQUAD_FILTER_NOTCH, q);
    }

    void initBiquadBpf()
    {
      float q = getNotchQ(_freq, _cutoff);
      initBiquad(BIQUAD_FILTER_BPF, q);
    }

    void initBiquadLpf()
    {
      initBiquad(BIQUAD_FILTER_LPF, 1.0f / sqrtf(2.0f)); /* quality factor - butterworth for lpf */
    }

    void initBiquad(BiquadFilterType filterType, float q)
    {
      initBiquadCoefficients(filterType, q);

      // zero initial samples
      _state.bq.x1 = _state.bq.x2 = 0;
      _state.bq.y1 = _state.bq.y2 = 0;
    }

    void initBiquadCoefficients(BiquadFilterType filterType, float q)
    {
      const float omega = 2.0f * Math::pi() * _freq / _rate;
      const float sn = sinf(omega);
      const float cs = cosf(omega);
      const float alpha = sn / (2.0f * q);

      float b0 = 0, b1 = 0, b2 = 0, a0 = 0, a1 = 0, a2 = 0;

      switch (filterType)
      {
        case BIQUAD_FILTER_LPF:
          b0 = (1.f - cs) * 0.5f;
          b1 =  1.f - cs;
          b2 = (1.f - cs) * 0.5f;
          a0 =  1.f + alpha;
          a1 = -2.f * cs;
          a2 =  1.f - alpha;
          break;
        case BIQUAD_FILTER_NOTCH:
          b0 =  1.f;
          b1 = -2.f * cs;
          b2 =  1.f;
          a0 =  1.f + alpha;
          a1 = -2.f * cs;
          a2 =  1.f - alpha;
          break;
        case BIQUAD_FILTER_BPF:
          b0 =  alpha;
          b1 =  0;
          b2 = -alpha;
          a0 =  1.f + alpha;
          a1 = -2.f * cs;
          a2 =  1.f - alpha;
          break;
      }

      // precompute the coefficients
      _state.bq.b0 = b0 / a0;
      _state.bq.b1 = b1 / a0;
      _state.bq.b2 = b2 / a0;
      _state.bq.a1 = a1 / a0;
      _state.bq.a2 = a2 / a0;
    }

    float updateBiquadDF1(float v) ICACHE_RAM_ATTR
    {
      /* compute result */
      const float result =
        _state.bq.b0 * v +
        _state.bq.b1 * _state.bq.x1 +
        _state.bq.b2 * _state.bq.x2 -
        _state.bq.a1 * _state.bq.y1 -
        _state.bq.a2 * _state.bq.y2;

      /* shift x1 to x2, input to x1 */
      _state.bq.x2 = _state.bq.x1;
      _state.bq.x1 = v;

      /* shift y1 to y2, result to y1 */
      _state.bq.y2 = _state.bq.y1;
      _state.bq.y1 = result;

      return result;
    }

    float updateBiquadDF2(float v) ICACHE_RAM_ATTR
    {
      const float result = _state.bq.b0 * v + _state.bq.x1;
      _state.bq.x1 = _state.bq.b1 * v - _state.bq.a1 * result + _state.bq.x2;
      _state.bq.x2 = _state.bq.b2 * v - _state.bq.a2 * result;
      return result;
    }

    FilterType _type;
    int _freq;
    int _rate;
    int _cutoff;
    union {
      FilterStatePt1 pt1;
      FilterStateBiquad bq;
      FilterStateMedian median;
    } _state;
};

}

#endif
