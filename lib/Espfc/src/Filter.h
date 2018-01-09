#ifndef _ESPFC_FILTER_H_
#define _ESPFC_FILTER_H_

#include <cmath>
#include <algorithm>

namespace Espfc {

enum FilterType {
  FILTER_PT1,
  FILTER_BIQUAD,
  FILTER_FIR,
  FILTER_NOTCH,
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
    int8_t type;
    int16_t freq;
    int16_t cutoff;
};

struct FilterStatePt1 {
  float k;
  float v;
  float v0;
};

struct FilterStatePt2 {
  float k;
  float v[2];
};

struct FilterStateBiquad {
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
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
      _freq = std::min((int)config.freq, _rate / 2); // adj cut freq below nyquist rule
      _cutoff = std::min((int)config.cutoff, _rate / 2); // adj cut freq below nyquist rule
      _cutoff = std::min(_cutoff, _freq - 10); // sanitize cutoff to be slightly below filter freq
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
          return updateBiquad(v);
        case FILTER_FIR:
          return updatePt1(v);
        case FILTER_NOTCH:
          return updateBiquad(v);
        case FILTER_NONE:
        default:
          return v;
      }
    }

  private:
    void initPt1()
    {
      float rc = 1.f / (2.f * M_PI * _freq);
      float dt = 1.f / _rate;
      _state.pt1.k = dt / (dt + rc);
      _state.pt1.v = 0.f;
      _state.pt1.v0 = 0.f;
    }

    float updatePt1(float v)
    {
      _state.pt1.v = _state.pt1.v + _state.pt1.k * (v - _state.pt1.v);
      _state.pt1.v = (_state.pt1.v + _state.pt1.v0) * 0.5f;
      _state.pt1.v0 = _state.pt1.v;
      return _state.pt1.v;
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
      const float omega = 2.0f * M_PI * _freq * _rate * 0.000001f;
      const float sn = sinf(omega);
      const float cs = cosf(omega);
      const float alpha = sn / (2.0f * q);

      float b0 = 0, b1 = 0, b2 = 0, a0 = 0, a1 = 0, a2 = 0;

      switch (filterType)
      {
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
      FilterStatePt2 pt2;
    } _state;
};

}

#endif
