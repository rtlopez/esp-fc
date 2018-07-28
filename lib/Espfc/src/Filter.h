#ifndef _ESPFC_FILTER_H_
#define _ESPFC_FILTER_H_

#include <Arduino.h>
//#include "math.h"
//#include <algorithm>
#include "ModelConfig.h"

// Quick median filter implementation
// (c) N. Devillard - 1998
// http://ndevilla.free.fr/median/median.pdf
#define QMF_SORT(a,b) { if ((a)>(b)) QMF_SWAP((a),(b)); }
#define QMF_SWAP(a,b) { int32_t temp=(a);(a)=(b);(b)=temp; }
#define QMF_COPY(p,v,n) { for (size_t i=0; i<n; i++) p[i]=v[i]; }
#define QMF_SORTF(a,b) { if ((a)>(b)) QMF_SWAPF((a),(b)); }
#define QMF_SWAPF(a,b) { float temp=(a);(a)=(b);(b)=temp; }

namespace Espfc {

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
      _freq = constrain((int)config.freq, 0, _rate / 2); // adj cut freq below nyquist rule
      _cutoff = constrain((int)config.cutoff, 0, _rate / 2); // adj cut freq below nyquist rule
      _cutoff = constrain(_cutoff, 0, _freq - 10); // sanitize cutoff to be slightly below filter freq
      if(_freq == 0) _type = FILTER_NONE; // turn off if filter freq equals zero
      switch(_type)
      {
        case FILTER_PT1:
          initPt1();
          break;
        case FILTER_BIQUAD:
          initBiquadLpf();
          break;
        case FILTER_FIR:
          initPt1();
          break;
        case FILTER_NOTCH:
          initBiquadNotch();
          break;
        case FILTER_FIR2:
          initFir2();
          break;
        case FILTER_MEDIAN3:
          initMedian3();
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
        case FILTER_FIR:
          return updatePt1Fir2(v);
        case FILTER_NOTCH:
          return updateBiquadDF2(v);
        case FILTER_FIR2:
          return updateFir2(v);
        case FILTER_MEDIAN3:
          return updateMedian3(v);
        case FILTER_NONE:
        default:
          return v;
      }
    }

  private:
    void initPt1()
    {
      float rc = 1.f / (2.f * PI * _freq);
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

    float updatePt1(float v) /* ICACHE_RAM_ATTR */
    {
      _state.pt1.v += _state.pt1.k * (v - _state.pt1.v);
      return _state.pt1.v;
    }

    float updatePt1Fir2(float v) /* ICACHE_RAM_ATTR */
    {
      _state.pt1.v += _state.pt1.k * (v - _state.pt1.v);
      _state.pt1.v = (_state.pt1.v + _state.pt1.v0) * 0.5f;
      _state.pt1.v0 = _state.pt1.v;
      return _state.pt1.v;
    }

    float updateFir2(float v) /* ICACHE_RAM_ATTR */
    {
      _state.pt1.v = (_state.pt1.v + _state.pt1.v0) * 0.5f;
      _state.pt1.v0 = _state.pt1.v;
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

    void initBiquadLpf()
    {
      initBiquad(BIQUAD_FILTER_LPF, 1.0f / sqrtf(2.0f)); /* quality factor - butterworth for lpf */
    }

    void initBiquad(BiquadFilterType filterType, float q)
    {
      const float omega = 2.0f * M_PI * _freq / _rate;
      const float sn = sinf(omega);
      const float cs = cosf(omega);
      const float alpha = sn / (2.0f * q);

      float b0 = 0, b1 = 0, b2 = 0, a0 = 0, a1 = 0, a2 = 0;

      switch (filterType)
      {
        case BIQUAD_FILTER_LPF:
          b0 = (1 - cs) * 0.5f;
          b1 =  1 - cs;
          b2 = (1 - cs) * 0.5f;
          a0 =  1 + alpha;
          a1 = -2 * cs;
          a2 =  1 - alpha;
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
          b0 =  alpha;
          b1 =  0;
          b2 = -alpha;
          a0 =  1 + alpha;
          a1 = -2 * cs;
          a2 =  1 - alpha;
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

    float updateBiquadDF1(float v) /* ICACHE_RAM_ATTR */
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

    float updateBiquadDF2(float v) /* ICACHE_RAM_ATTR */
    {
      const float result = _state.bq.b0 * v + _state.bq.x1;
      _state.bq.x1 = _state.bq.b1 * v - _state.bq.a1 * result + _state.bq.x2;
      _state.bq.x2 = _state.bq.b2 * v - _state.bq.a2 * result;
      return result;
    }

    float getNotchQApprox(int freq, int cutoff)
    {
      return ((float)(cutoff * freq) / ((float)(freq - cutoff) * (float)(freq + cutoff)));
    }

    float getNotchQ(int freq, int cutoff)
    {
      float octaves = log2f((float)freq  / (float)cutoff) * 2;
      return sqrtf(powf(2, octaves)) / (powf(2, octaves) - 1);
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
