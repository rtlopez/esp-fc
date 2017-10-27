#ifndef _ESPFC_PID_H_
#define _ESPFC_PID_H_

#include "Math.h"
#include "Filter.h"

// bataflight scalers
//#define PTERM_SCALE 0.032029f
//#define ITERM_SCALE 0.244381f
//#define DTERM_SCALE 0.000529f

// espfc scalers
#define PTERM_SCALE 0.002f    // 1/500
#define ITERM_SCALE 0.002f    // 1/500
#define DTERM_SCALE 0.00004f  // 1/25000

#define LEVEL_PTERM_SCALE 0.1f    // 1/10
#define LEVEL_ITERM_SCALE 0.1f    // 1/10
#define LEVEL_DTERM_SCALE 0.001f  // 1/1000

namespace Espfc {

class PidConfig
{
  public:
    uint8_t P;
    uint8_t I;
    uint8_t D;
};

class Pid
{
  public:
    Pid(): Kp(0.1), Ki(0), Kd(0), iLimit(0), dGamma(0), oLimit(1.f), pScale(1.f), iScale(1.f), dScale(1.f) {}

    void configureFilter(FilterType type, int cutFreq, int sampleFreq)
    {
      dtermFilter.begin(type, cutFreq, sampleFreq);
    }

    void configurePid(float p, float i, float d, float il = 0.3, float dg = 0, float ol = 1)
    {
      Kp = p;
      Ki = i;
      Kd = d;
      iLimit = il;
      dGamma = dg;
      oLimit = ol;
    }

    float update(float setpoint, float measure, float dt)
    {
      error = setpoint - measure;
      pTerm = Kp * error * pScale;
      if(iScale > 0.01)
      {
        iTerm += Ki * error * dt * iScale;
        iTerm = Math::bound(iTerm, -iLimit, iLimit);
      }
      else
      {
        iTerm = 0; // zero integral
      }

      float delta = 0;
      if(Kd > 0 && dt > 0)
      {
        delta = (Kd * dScale * (((error - prevError) * dGamma) + (prevMeasure - measure) * (1.f - dGamma)) / dt);
      }
      dTerm = dtermFilter.update(delta);

      float output = Math::bound(pTerm + iTerm + dTerm, -oLimit, oLimit);

      prevMeasure = measure;
      prevError = error;

      return output;
    }

    float Kp;
    float Ki;
    float Kd;

    float iLimit;
    float dGamma;
    float oLimit;

    float pScale;
    float iScale;
    float dScale;
    Filter dtermFilter;

    float error;
    float pTerm;
    float iTerm;
    float dTerm;
    float prevMeasure;
    float prevError;
};

}

#endif
