#pragma once

#include <cstdint>
#include <cstddef>
#include "Utils/Math.hpp"

namespace Espfc {

enum FilterType {
  FILTER_PT1,
  FILTER_BIQUAD,
  FILTER_PT2,
  FILTER_PT3,
  FILTER_NOTCH,
  FILTER_NOTCH_DF1,
  FILTER_BPF,
  FILTER_FO,
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
    FilterConfig();
    FilterConfig(FilterType t, int16_t f, int16_t c = 0);
    FilterConfig sanitize(int rate) const;

    int8_t type;
    int16_t freq;
    int16_t cutoff;
};

constexpr size_t DYN_NOTCH_COUNT_MAX = 6;

class DynamicFilterConfig {
  public:
    DynamicFilterConfig() {}
    DynamicFilterConfig(int8_t c, int16_t qf, int16_t lf, int16_t hf): count(c), q(qf), min_freq(lf), max_freq(hf) {}
    uint8_t count = 4;
    int16_t q = 300;
    int16_t min_freq = 80;
    int16_t max_freq = 400;
    static constexpr int MIN_FREQ = 1000;
};

namespace Utils {

inline float pt1Gain(float rate, float freq)
{
  float rc = 1.f / (2.f * pi() * freq);
  float dt = 1.f / rate;
  return dt / (dt + rc);
}

class FilterStatePt1 {
  public:
    void reset();
    void reconfigure(const FilterStatePt1& from);
    void init(float rate, float freq);
    float update(float n);

    float k;
    float v;
};

class FilterStateFir2 {
  public:
    void reset();
    void init();
    void reconfigure(const FilterStateFir2& from);
    float update(float n);

    float v[2];
};

class FilterStateBiquad {
  public:
    void reset();
    void init(BiquadFilterType filterType, float rate, float freq, float q);
    void reconfigure(const FilterStateBiquad& from);
    float update(float n);
    float updateDF1(float n);

    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
};

class FilterStateFirstOrder {
  public:
    void reset();
    void init(float rate, float freq);
    void reconfigure(const FilterStateFirstOrder& from);
    float update(float n);
    float updateDF1(float n);

    float b0, b1, a1;
    float x1, y1;
};

class FilterStateMedian {
  public:
    void reset();
    void init();
    void reconfigure(const FilterStateMedian& from);
    float update(float n);

    float v[3];
};

class FilterStatePt2 {
  public:
    void reset();
    void init(float rate, float freq);
    void reconfigure(const FilterStatePt2& from);
    float update(float n);

    float k;
    float v[2];
};

class FilterStatePt3 {
  public:
    void reset();
    void init(float rate, float freq);
    void reconfigure(const FilterStatePt3& from);
    float update(float n);

    float k;
    float v[3];
};

class Filter
{
  public:
    Filter();
    void begin();
    void begin(const FilterConfig& config, int rate);
    float update(float v);
    void reset();

    void reconfigure(int16_t freq, int16_t cutoff = 0);
    void reconfigure(int16_t freq, int16_t cutoff, float q, float weight = 1.0f);
    void reconfigure(const FilterConfig& config, int rate);
    void reconfigure(const FilterConfig& config, int rate, float q, float weight);
    void reconfigure(const Filter& filter);
    void setWeight(float weight);
    float getNotchQApprox(float freq, float cutoff);
    float getNotchQ(float freq, float cutoff);

#if !defined(UNIT_TEST)
  private:
#endif

    int _rate;
    FilterConfig _conf;
    union {
      FilterStatePt1 pt1;
      FilterStateBiquad bq;
      FilterStateFir2 fir2;
      FilterStateMedian median;
      FilterStatePt2 pt2;
      FilterStatePt3 pt3;
      FilterStateFirstOrder fo;
    } _state;
    float _input_weight;
    float _output_weight;
};

}

}