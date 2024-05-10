#include <cmath>
#include "Filter.h"
#include "Math/Utils.h"
#include "Utils/MemoryHelper.h"

// Quick median filter implementation
// (c) N. Devillard - 1998
// http://ndevilla.free.fr/median/median.pdf
#define QMF_SORT(a,b) { if ((a)>(b)) QMF_SWAP((a),(b)); }
#define QMF_SWAP(a,b) { int32_t temp=(a);(a)=(b);(b)=temp; }
#define QMF_COPY(p,v,n) { for (size_t i=0; i<n; i++) p[i]=v[i]; }
#define QMF_SORTF(a,b) { if ((a)>(b)) QMF_SWAPF((a),(b)); }
#define QMF_SWAPF(a,b) { float temp=(a);(a)=(b);(b)=temp; }

static float pt1Gain(float rate, float freq)
{
  float rc = 1.f / (2.f * Espfc::Math::pi() * freq);
  float dt = 1.f / rate;
  return dt / (dt + rc);
}

namespace Espfc {

FilterConfig::FilterConfig(): type(FILTER_NONE), freq(0), cutoff(0) {}
FilterConfig::FilterConfig(FilterType t, int16_t f, int16_t c): type(t), freq(f), cutoff(c) {}

FilterConfig FAST_CODE_ATTR FilterConfig::sanitize(int rate) const
{
  const int halfRate = rate * 0.49f;
  FilterType t = (FilterType)type;
  int16_t f = Math::clamp((int)freq, 0, halfRate);   // adj cut freq below nyquist rule
  int16_t c = Math::clamp((int)cutoff, 0, (int)(f * 0.98f));      // sanitize cutoff to be slightly below filter freq

  bool biquad = type == FILTER_NOTCH || type == FILTER_NOTCH_DF1 || type == FILTER_BPF;
  if(f == 0 || (biquad && c == 0)) t = FILTER_NONE; // if freq is zero or cutoff for biquad, turn off

  return FilterConfig(t, f, c);
}

void FilterStatePt1::reset()
{
  v = 0.f;
}

void FAST_CODE_ATTR FilterStatePt1::reconfigure(const FilterStatePt1& from)
{
  k = from.k;
}

void FilterStatePt1::init(float rate, float freq)
{
  k = pt1Gain(rate, freq);
}

float FAST_CODE_ATTR FilterStatePt1::update(float n)
{
  v += k * (n - v);
  return v;
}

void FilterStateFir2::reset()
{
  v[0] = v[1] = 0.0f;
}

void FilterStateFir2::init()
{
}

void FAST_CODE_ATTR FilterStateFir2::reconfigure(const FilterStateFir2& from)
{
}

float FAST_CODE_ATTR FilterStateFir2::update(float n)
{
  v[0] = (n + v[1]) * 0.5f;
  v[1] = n;
  return v[0];
}

void FilterStateBiquad::reset()
{
  x1 = x2 = y1 = y2 = 0;
}

void FilterStateBiquad::init(BiquadFilterType filterType, float rate, float freq, float q)
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

void FAST_CODE_ATTR FilterStateBiquad::reconfigure(const FilterStateBiquad& from)
{
  b0 = from.b0;
  b1 = from.b1;
  b2 = from.b2;
  a1 = from.a1;
  a2 = from.a2;
}

float FAST_CODE_ATTR FilterStateBiquad::update(float n)
{
  // DF2
  const float result = b0 * n + x1;
  x1 = b1 * n - a1 * result + x2;
  x2 = b2 * n - a2 * result;
  return result;
}

float FAST_CODE_ATTR FilterStateBiquad::updateDF1(float n)
{
  /* compute result */
  const float result = b0 * n + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

  /* shift x1 to x2, input to x1 */
  x2 = x1; x1 = n;

  /* shift y1 to y2, result to y1 */
  y2 = y1; y1 = result;

  return result;
}

void FilterStateFirstOrder::reset()
{
  x1 = y1 = 0;
}

void FilterStateFirstOrder::init(float rate, float freq)
{
  freq = Math::clamp(freq, 0.0f, rate * 0.48f);

  const float W = std::tan(Math::pi() * freq / rate);

  a1 = (W - 1) / (W + 1);
  b1 = b0 = W / (W + 1);
}

void FAST_CODE_ATTR FilterStateFirstOrder::reconfigure(const FilterStateFirstOrder& from)
{
  b0 = from.b0;
  b1 = from.b1;
  a1 = from.a1;
}

float FAST_CODE_ATTR FilterStateFirstOrder::update(float n)
{
  // DF2
  const float result = b0 * n + x1;
  x1 = b1 * n - a1 * result;
  return result;
}

float FAST_CODE_ATTR FilterStateFirstOrder::updateDF1(float n)
{
  /* compute result */
  const float result = b0 * n + b1 * x1 - a1 * y1;

  /* shift input to x1 */
  x1 = n;

  /* shift result to y1 */
  y1 = result;

  return result;
}

void FilterStateMedian::reset()
{
  v[0] = v[1] = v[2] = 0.f;
}

void FilterStateMedian::init()
{
}

void FAST_CODE_ATTR FilterStateMedian::reconfigure(const FilterStateMedian& from)
{
}

float FAST_CODE_ATTR FilterStateMedian::update(float n)
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

void FilterStatePt2::reset()
{
  v[0] = v[1] = 0.f;
}

void FilterStatePt2::init(float rate, float freq)
{
  constexpr float correction = 1.553773974f; // 1 / sqrt(2^(1/n) - 1)
  k = pt1Gain(rate, freq * correction);
}

void FAST_CODE_ATTR FilterStatePt2::reconfigure(const FilterStatePt2& from)
{
  k = from.k;
}

float FAST_CODE_ATTR FilterStatePt2::update(float n)
{
  v[0] += k * (n - v[0]);
  v[1] += k * (v[0] - v[1]);
  return v[1];
}

void FilterStatePt3::reset()
{
  v[0] = v[1] = v[2] = 0.f;
}

void FilterStatePt3::init(float rate, float freq)
{
  constexpr float correction = 1.961459177f; // 1 / sqrt(2^(1/n) - 1)
  k = pt1Gain(rate, freq * correction);
}

void FAST_CODE_ATTR FilterStatePt3::reconfigure(const FilterStatePt3& from)
{
  k = from.k;
}

float FAST_CODE_ATTR FilterStatePt3::update(float n)
{
  v[0] += k * (n - v[0]);
  v[1] += k * (v[0] - v[1]);
  v[2] += k * (v[1] - v[2]);
  return v[2];
}

Filter::Filter(): _conf(FilterConfig(FILTER_NONE, 0)) {}

void Filter::begin()
{
  _conf = FilterConfig(FILTER_NONE, 0);
}

void Filter::begin(const FilterConfig& config, int rate)
{
  reconfigure(config, rate);
  reset();
}

float FAST_CODE_ATTR Filter::update(float v)
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
      return _output_weight * _state.bq.updateDF1(v) + _input_weight * v;
    case FILTER_FIR2:
      return _state.fir2.update(v);
    case FILTER_MEDIAN3:
      return _state.median.update(v);
    case FILTER_PT2:
      return _state.pt2.update(v);
    case FILTER_PT3:
      return _state.pt3.update(v);
    case FILTER_FO:
      return _state.fo.update(v);
    case FILTER_NONE:
    default:
      return v;
  }
}

void Filter::reset()
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
    case FILTER_PT2:
      return _state.pt2.reset();
    case FILTER_PT3:
      return _state.pt3.reset();
    case FILTER_FO:
      return _state.fo.reset();
    case FILTER_NONE:
    default:
      ;
  }
}

void FAST_CODE_ATTR Filter::reconfigure(int16_t freq, int16_t cutoff)
{
  reconfigure(FilterConfig((FilterType)_conf.type, freq, cutoff), _rate);
}

void FAST_CODE_ATTR Filter::reconfigure(int16_t freq, int16_t cutoff, float q, float weight)
{
  reconfigure(FilterConfig((FilterType)_conf.type, freq, cutoff), _rate, q, weight);
}

void FAST_CODE_ATTR Filter::reconfigure(const FilterConfig& config, int rate)
{
  _rate = rate;
  _conf = config.sanitize(_rate);
  switch(_conf.type)
  {
    case FILTER_BIQUAD:
      reconfigure(config, rate, 0.70710678118f, 1.0f); // 1.0f / sqrtf(2.0f); // quality factor for butterworth lpf
      break;
    case FILTER_NOTCH:
    case FILTER_NOTCH_DF1:
    case FILTER_BPF:
      reconfigure(config, rate, getNotchQApprox(config.freq, config.cutoff), 1.0f);
      break;
    default:
      reconfigure(config, rate, 0.0f, 1.0f);
  }
}

void FAST_CODE_ATTR Filter::reconfigure(const FilterConfig& config, int rate, float q, float weight)
{
  _rate = rate;
  _conf = config.sanitize(_rate);
  setWeight(weight);
  switch(_conf.type)
  {
    case FILTER_PT1:
      _state.pt1.init(_rate, _conf.freq);
      break;
    case FILTER_BIQUAD:
      _state.bq.init(BIQUAD_FILTER_LPF, _rate, _conf.freq, q);
      break;
    case FILTER_NOTCH:
    case FILTER_NOTCH_DF1:
      _state.bq.init(BIQUAD_FILTER_NOTCH, _rate, _conf.freq, q);
      break;
    case FILTER_BPF:
      _state.bq.init(BIQUAD_FILTER_BPF, _rate, _conf.freq, q);
      break;
    case FILTER_FIR2:
      _state.fir2.init();
      break;
    case FILTER_MEDIAN3:
      _state.median.init();
      break;
    case FILTER_PT2:
      _state.pt2.init(_rate, _conf.freq);
      break;
    case FILTER_PT3:
      _state.pt3.init(_rate, _conf.freq);
      break;
    case FILTER_FO:
      _state.fo.init(_rate, _conf.freq);
      break;
    case FILTER_NONE:
    default:
      ;
  }
}

void FAST_CODE_ATTR Filter::reconfigure(const Filter& filter)
{
  _rate = filter._rate;
  _conf = filter._conf;
  _output_weight = filter._output_weight;
  _input_weight = filter._input_weight;
  switch(_conf.type)
  {
    case FILTER_PT1:
      _state.pt1.reconfigure(filter._state.pt1);
      break;
    case FILTER_BIQUAD:
    case FILTER_NOTCH:
    case FILTER_NOTCH_DF1:
      _state.bq.reconfigure(filter._state.bq);
      break;
    case FILTER_FIR2:
      _state.fir2.reconfigure(filter._state.fir2);
      break;
    case FILTER_MEDIAN3:
      _state.median.reconfigure(filter._state.median);
      break;
    case FILTER_PT2:
      _state.pt2.reconfigure(filter._state.pt2);
      break;
    case FILTER_PT3:
      _state.pt3.reconfigure(filter._state.pt3);
      break;
    case FILTER_FO:
      _state.fo.reconfigure(filter._state.fo);
      break;
    case FILTER_NONE:
    default:
      ;
  }
}

void FAST_CODE_ATTR Filter::setWeight(float weight)
{
  _output_weight = std::max(0.0f, std::min(weight, 1.0f));
  _input_weight = 1.0f - _output_weight;
}

float FAST_CODE_ATTR Filter::getNotchQApprox(float freq, float cutoff)
{
  return ((float)(cutoff * freq) / ((float)(freq - cutoff) * (float)(freq + cutoff)));
}

float FAST_CODE_ATTR Filter::getNotchQ(float freq, float cutoff)
{
  float octaves = std::log2(freq / cutoff) * 2.f;
  return sqrtf(std::pow(2.f, octaves)) / (std::pow(2.f, octaves) - 1);
}

}
