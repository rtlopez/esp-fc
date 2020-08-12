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
  FILTER_NOTCH,
  FILTER_NOTCH_DF1,
  FILTER_BPF,
  FILTER_FIR2,
  FILTER_MEDIAN3,
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
    FilterConfig(): type(FILTER_NONE), freq(0), cutoff(0) {}
    FilterConfig(FilterType t, int16_t f, int16_t c = 0): type(t), freq(f), cutoff(c) {}

    FilterConfig sanitize(int rate) const
    {
      const int halfRate = rate / 2;
      FilterType t = (FilterType)type;
      int16_t f = Math::clamp((int)freq, 0, halfRate);   // adj cut freq below nyquist rule
      int16_t c = Math::clamp((int)cutoff, 0, (int)(f * 0.98f));      // sanitize cutoff to be slightly below filter freq

      bool biquad = type == FILTER_NOTCH || type == FILTER_NOTCH_DF1 || type == FILTER_BPF;
      if(f == 0 || (biquad && c == 0)) t = FILTER_NONE; // if freq is zero or cutoff for biquad, turn off

      return FilterConfig(t, f, c);
    }

    FilterConfig reconfigure(int16_t freq, int16_t cutoff = 0) const
    {
      return FilterConfig((FilterType)type, freq, cutoff);
    }

    int8_t type;
    int16_t freq;
    int16_t cutoff;
};

class DynamicFilterConfig {
  public:
    DynamicFilterConfig() {}
    DynamicFilterConfig(int8_t w, int16_t qf, int16_t lf, int16_t hf): width(w), q(qf), min_freq(lf), max_freq(hf) {}
    int8_t width;
    int16_t q;
    int16_t min_freq;
    int16_t max_freq;
};

class FilterStatePt1 {
  public:
    void reset()
    {
      v = 0.f;
    }

    void init(float rate, float freq)
    {
      float rc = 1.f / (2.f * Math::pi() * freq);
      float dt = 1.f / rate;
      k = dt / (dt + rc);
    }

    float update(float n)
    {
      v += k * (n - v);
      return v;
    }

    float k;
    float v;
};

class FilterStateFir2 {
  public:
    void reset()
    {
      v[0] = v[1] = 0.0f;
    }

    void init()
    {
    }

    float update(float n)
    {
      v[0] = (n + v[1]) * 0.5f;
      v[1] = n;
      return v[0];
    }

    float v[2];
};

class FilterStateBiquad {
  public:
    void reset()
    {
      x1 = x2 = y1 = y2 = 0;
    }

    void init(BiquadFilterType filterType, float rate, float freq, float q)
    {
      const float omega = (2.0f * Math::pi() * freq) / rate;
      const float sn = sinf(omega);
      const float cs = cosf(omega);
      const float alpha = sn / (2.0f * q);
      float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f, a0 = 0.0f, a1 = 0.0f, a2 = 0.0f;
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
      this->b0 = b0 / a0;
      this->b1 = b1 / a0;
      this->b2 = b2 / a0;
      this->a1 = a1 / a0;
      this->a2 = a2 / a0;
    }

    float update(float n)
    {
      // DF2
      const float result = b0 * n + x1;
      x1 = b1 * n - a1 * result + x2;
      x2 = b2 * n - a2 * result;
      return result;
    }

    float updateDF1(float n)
    {
      /* compute result */
      const float result = b0 * n + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

      /* shift x1 to x2, input to x1 */
      x2 = x1; x1 = n;

      /* shift y1 to y2, result to y1 */
      y2 = y1; y1 = result;

      return result;
    }

    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
};

class FilterStateMedian {
  public:
    void reset()
    {
      v[0] = v[1] = v[2] = 0.f;
    }

    void init()
    {

    }

    float update(float n)
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = n;
      float p[3];
      QMF_COPY(p, v, 3);
      QMF_SORTF(p[0], p[1]);
      QMF_SORTF(p[1], p[2]);
      QMF_SORTF(p[0], p[1]);
      return p[1];
    }

    float v[3];
};


class Filter
{
  public:
    Filter(): _conf(FilterConfig(FILTER_NONE, 0)) {}

    void begin()
    {
      _conf = FilterConfig(FILTER_NONE, 0);
    }

    void begin(const FilterConfig& config, int rate)
    {
      reconfigure(config, rate);
      reset();
    }

    float update(float v)
    {
      switch(_conf.type)
      {
        case FILTER_PT1:
          return _state.pt1.update(v);
        case FILTER_BIQUAD:
        case FILTER_NOTCH:
        case FILTER_BPF:
          return _state.bq.update(v);
        case FILTER_NOTCH_DF1:
          return _state.bq.updateDF1(v);
        case FILTER_FIR2:
          return _state.fir2.update(v);
        case FILTER_MEDIAN3:
          return _state.median.update(v);
        case FILTER_NONE:
        default:
          return v;
      }
    }

    void reset()
    {
      switch(_conf.type)
      {
        case FILTER_PT1:
          _state.pt1.reset();
          break;
        case FILTER_BIQUAD:
        case FILTER_NOTCH:
        case FILTER_NOTCH_DF1:
        case FILTER_BPF:
          _state.bq.reset();
          break;
        case FILTER_FIR2:
          _state.fir2.reset();
          break;
        case FILTER_MEDIAN3:
          _state.median.reset();
          break;
        case FILTER_NONE:
        default:
          ;
      }
    }

    void reconfigure(int16_t freq, int16_t cutoff = 0)
    {
      reconfigure(_conf.reconfigure(freq, cutoff), _rate);
    }

    void reconfigure(const FilterConfig& config, int rate)
    {
      _rate = rate;
      _conf = config.sanitize(_rate);
      switch(_conf.type)
      {
        case FILTER_PT1:
          _state.pt1.init(_rate, _conf.freq);
          break;
        case FILTER_BIQUAD:
          initBiquadLpf(_rate, _conf.freq);
          break;
        case FILTER_NOTCH:
        case FILTER_NOTCH_DF1:
          initBiquadNotch(_rate, _conf.freq, _conf.cutoff);
          break;
        case FILTER_BPF:
          initBiquadBpf(_rate, _conf.freq, _conf.cutoff);
          break;
        case FILTER_FIR2:
          _state.fir2.init();
          break;
        case FILTER_MEDIAN3:
          _state.median.init();
          break;
        case FILTER_NONE:
        default:
          ;
      }
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

#if !defined(UNIT_TEST)
  private:
#endif
    // BIQUAD
    void initBiquadLpf(float rate, float freq)
    {
      //const float q = 0.70710678118f;
      const float q = 1.0f / sqrtf(2.0f); /* quality factor: butterworth for lpf */
      _state.bq.init(BIQUAD_FILTER_LPF, rate, freq, q);
    }

    void initBiquadNotch(float rate, float freq, float cutoff)
    {
      const float q = getNotchQApprox(freq, cutoff);
      _state.bq.init(BIQUAD_FILTER_NOTCH, rate, freq, q);
    }

    void initBiquadBpf(float rate, float freq, float cutoff)
    {
      const float q = getNotchQApprox(freq, cutoff);
      _state.bq.init(BIQUAD_FILTER_BPF, rate, freq, q);
    }

    int _rate;
    FilterConfig _conf;
    union {
      FilterStatePt1 pt1;
      FilterStateBiquad bq;
      FilterStateFir2 fir2;
      FilterStateMedian median;
    } _state;
};

}

#endif
