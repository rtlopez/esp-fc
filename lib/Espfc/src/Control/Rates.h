#ifndef _ESPFC_RATES_H_
#define _ESPFC_RATES_H_

#include "Math/Utils.h"

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
    void begin(const InputConfig& config)
    {
      rateType = (RateType)config.rateType;
      for(size_t i = 0; i < 3; i++)
      {
        rcExpo[i] = config.expo[i];
        rcRates[i] = config.rate[i];
        rates[i] = config.superRate[i];
        rateLimit[i] = config.rateLimit[i];
      }
    }

    float getSetpoint(const int axis, float input)
    {
      input = Math::clamp(input, -0.995f, 0.995f); // limit input
      const float inputAbs = fabsf(input);
      float result = 0;
      switch(rateType)
      {
        case RATES_TYPE_BETAFLIGHT:
          result = betaflight(axis, input, inputAbs);
        case RATES_TYPE_RACEFLIGHT:
          result = raceflight(axis, input, inputAbs);
        case RATES_TYPE_KISS:
          result = kiss(axis, input, inputAbs);
        case RATES_TYPE_ACTUAL:
          result = actual(axis, input, inputAbs);
        case RATES_TYPE_QUICK:
          result = quick(axis, input, inputAbs);
      }
      return Math::toRad(Math::clamp(result, -(float)rateLimit[axis], (float)rateLimit[axis]));
    }

  private:
    float betaflight(const int axis, float rcCommandf, const float rcCommandfAbs)
    {
      if (this->rcExpo[axis])
      {
        const float expof = this->rcExpo[axis] / 100.0f;
        rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
      }

      float rcRate = this->rcRates[axis] / 100.0f;
      if (rcRate > 2.0f)
      {
        rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
      }
      float angleRate = 200.0f * rcRate * rcCommandf;
      if (this->rates[axis])
      {
        const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (this->rates[axis] / 100.0f)), 0.01f, 1.00f));
        angleRate *= rcSuperfactor;
      }

      return angleRate;
    }

    float raceflight(const int axis, float rcCommandf, const float rcCommandfAbs)
    {
      // -1.0 to 1.0 ranged and curved
      rcCommandf = ((1.0f + 0.01f * this->rcExpo[axis] * (rcCommandf * rcCommandf - 1.0f)) * rcCommandf);
      // convert to -2000 to 2000 range using acro+ modifier
      float angleRate = 10.0f * this->rcRates[axis] * rcCommandf;
      angleRate = angleRate * (1 + rcCommandfAbs * (float)this->rates[axis] * 0.01f);

      return angleRate;
    }

    float kiss(const int axis, float rcCommandf, const float rcCommandfAbs)
    {
      const float rcCurvef = this->rcExpo[axis] / 100.0f;

      float kissRpyUseRates = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (this->rates[axis] / 100.0f)), 0.01f, 1.00f));
      float kissRcCommandf = (power3(rcCommandf) * rcCurvef + rcCommandf * (1 - rcCurvef)) * (this->rcRates[axis] / 1000.0f);
      float kissAngle = constrainf(((2000.0f * kissRpyUseRates) * kissRcCommandf), -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT);

      return kissAngle;
    }

    float actual(const int axis, float rcCommandf, const float rcCommandfAbs)
    {
      float expof = this->rcExpo[axis] / 100.0f;
      expof = rcCommandfAbs * (power5(rcCommandf) * expof + rcCommandf * (1 - expof));

      const float centerSensitivity = this->rcRates[axis] * 10.0f;
      const float stickMovement = std::max(0.f, this->rates[axis] * 10.0f - centerSensitivity);
      const float angleRate = rcCommandf * centerSensitivity + stickMovement * expof;

      return angleRate;
    }

    float quick(const int axis, float rcCommandf, const float rcCommandfAbs)
    {
      const float rcRate = this->rcRates[axis] * 2;
      const float maxDPS = std::max(this->rates[axis] * 10.f, rcRate);
      const float linearity = this->rcExpo[axis] / 100.0f;
      const float superFactorConfig = (maxDPS / rcRate - 1) / (maxDPS / rcRate);

      float curve = power3(rcCommandfAbs) * linearity + rcCommandfAbs * (1 - linearity);
      float superfactor = 1.0f / (constrainf(1.0f - (curve * superFactorConfig), 0.01f, 1.00f));
      float angleRate = constrainf(rcCommandf * rcRate * superfactor, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT);

      return angleRate;
    }

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

#endif
