#ifndef _ESPFC_PID_H_
#define _ESPFC_PID_H_

#include "ModelConfig.h"
#include "Math.h"
#include "Filter.h"

// bataflight scalers
//#define PTERM_SCALE 0.032029f
//#define ITERM_SCALE 0.244381f
//#define DTERM_SCALE 0.000529f

// espfc scalers
#define PTERM_SCALE 0.0035f  // 0.005f // 1/200    // prev: 1/500
#define ITERM_SCALE 0.01f    // 0.005f   // 1/200    // prev: 1/500
#define DTERM_SCALE 0.00004f // 0.00005f // 1/20000  // prev: 1/25000

#define LEVEL_PTERM_SCALE 0.1f    // 1/10
#define LEVEL_ITERM_SCALE 0.1f    // 1/10
#define LEVEL_DTERM_SCALE 0.001f  // 1/1000

namespace Espfc {

class Pid
{
  public:
    Pid(): Kp(0.1), Ki(0), Kd(0), iLimit(0), dGamma(0), oLimit(1.f), pScale(1.f), iScale(1.f), dScale(1.f), iTerm(0), dTerm(0) {}

    void configure(float p, float i, float d, float il = 0.3, float dg = 0.f, float ol = 1.f)
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
      pTerm = ptermFilter.update(pTerm);

      if(iScale > 0.01)
      {
        if(std::abs(pTerm) < oLimit)
        {
          iTerm += Ki * error * dt * iScale;
        }
        iTerm = Math::bound(iTerm, -iLimit, iLimit);
      }
      else
      {
        iTerm = 0; // zero integral
      }

      if(Kd > 0 && dt > 0)
      {
        dTerm = (Kd * dScale * (((error - prevError) * dGamma) + (prevMeasure - measure) * (1.f - dGamma)) / dt);
      }
      dTerm = dtermNotchFilter.update(dTerm);
      dTerm = dtermFilter.update(dTerm);

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
    Filter dtermNotchFilter;
    Filter ptermFilter;

    float error;
    float pTerm;
    float iTerm;
    float dTerm;
    float prevMeasure;
    float prevError;
};

}

#endif
