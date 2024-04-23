#pragma once

#include "Math/Utils.h"
#include "ModelConfig.h"

#define SETPOINT_RATE_LIMIT 1998.0f
#define RC_RATE_INCREMENTAL 14.54f

namespace Espfc
{

enum RateType {
  RATES_TYPE_BETAFLIGHT = 0,
  RATES_TYPE_RACEFLIGHT,
  RATES_TYPE_KISS,
  RATES_TYPE_ACTUAL,
  RATES_TYPE_QUICK,
};

class Rates
{
  public:
    void begin(const InputConfig& config);
    float getSetpoint(const int axis, float input) const;

  private:
    float betaflight(const int axis, float rcCommandf, const float rcCommandfAbs) const;
    float raceflight(const int axis, float rcCommandf, const float rcCommandfAbs) const;
    float kiss(const int axis, float rcCommandf, const float rcCommandfAbs) const;
    float actual(const int axis, float rcCommandf, const float rcCommandfAbs) const;
    float quick(const int axis, float rcCommandf, const float rcCommandfAbs) const;

    inline float power3(float x) const
    {
      return x * x * x;
    }

    inline float power5(float x) const
    {
      return x * x * x * x * x;
    }

    inline float constrainf(float x, float l, float h) const
    {
      return Math::clamp(x, l, h);
    }

  private:
    RateType rateType;
    uint8_t rcExpo[3];
    uint8_t rcRates[3];
    uint8_t rates[3];
    int16_t rateLimit[3];
};

} // namespace Espfc
